#include "plc_persist.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include "plc.h"

// TinyCBOR
#include "cbor.h"
#include "logf.h"
#include "FreeRTOS.h"
#include "task.h"

#define TAG = "PERSIST";
static const char *tag = "PERSIST";
// Короткое имя кода ошибки CBOR
static const char* _cbor_err(CborError e){
    switch(e){
        case CborNoError: return "OK";
        case CborErrorOutOfMemory: return "OutOfMemory";
        case CborErrorUnknownType: return "UnknownType";
        case CborErrorIllegalNumber: return "IllegalNumber";
        case CborErrorTooManyItems: return "TooManyItems";
        case CborErrorDataTooLarge: return "DataTooLarge";
        case CborErrorIO: return "IO";
        case CborErrorInvalidUtf8TextString: return "BadUtf8";
        case CborErrorUnexpectedEOF: return "UnexpectedEOF";
        case CborErrorMapKeyNotString: return "MapKeyNotString";
        default: return "Other";
    }
}

static inline bool _ok(CborError e){
    if (e != CborNoError) LOGT(tag, "CBOR error: %s (%d)", _cbor_err(e), (int)e);
    return e == CborNoError;
}

static void _log_flash_sr(uint32_t sr) {
    LOGT(tag, "FLASH->SR=0x%08lX%s%s%s%s%s%s%s%s",
         (unsigned long)sr,
         (sr & FLASH_SR_BSY)    ? " BSY"    : "",
         (sr & FLASH_SR_EOP)    ? " EOP"    : "",
//         (sr & FLASH_SR_OPERR)  ? " OPERR"  : "",
         (sr & FLASH_SR_WRPERR) ? " WRPERR" : "",
         (sr & FLASH_SR_PGAERR) ? " PGAERR" : "",
         (sr & FLASH_SR_PGPERR) ? " PGPERR" : "",
         (sr & FLASH_SR_PGSERR) ? " PGSERR" : "",
         #if defined(FLASH_SR_RDERR)
         (sr & FLASH_SR_RDERR)  ? " RDERR"  : ""
#else
            ""
#endif
    );
}

static void _log_optcr(void) {
#if defined(FLASH_OPTCR)
    uint32_t opt = FLASH->OPTCR;
    LOGT(tag, "FLASH->OPTCR=0x%08lX (OPTLOCK=%lu, WRP=%lu)",
         (unsigned long)opt,
         (unsigned long)((opt & FLASH_OPTCR_OPTLOCK) ? 1 : 0),
         (unsigned long)((opt >> 16) & 0xFFFF));
#endif
}

static bool _addr_in_sector(uint32_t addr, uint32_t len) {
    uint32_t start = PLC_FLASH_SECTOR_ADDR;
    uint32_t end   = PLC_FLASH_SECTOR_ADDR + PLC_FLASH_SECTOR_SIZE;
    uint32_t aend  = addr + len;
    if (addr < start || aend > end) return false;
    return true;
}

static bool _probe_ff(uint32_t addr, size_t bytes) {
    const uint8_t* p = (const uint8_t*)addr;
    for (size_t i=0;i<bytes;i++) if (p[i] != 0xFF) return false;
    return true;
}

static void _hexdump_at(uint32_t addr, size_t len, size_t max) {
    const uint8_t* p = (const uint8_t*)addr;
    size_t n = (len < max) ? len : max;
    for (size_t i=0;i<n;i++){
        if ((i & 0x0F)==0) printf("\r\n%08lX: ", (unsigned long)(addr + i));
        printf("%02X ", p[i]);
    }
    printf("\r\n");
}


// ===== CRC32 (IEEE 802.3, poly 0x04C11DB7, init 0xFFFFFFFF, final XOR 0xFFFFFFFF) =====
static uint32_t crc32_update(uint32_t crc, const uint8_t* p, size_t n){
    crc = ~crc;
    for(size_t i=0;i<n;i++){
        crc ^= (uint32_t)p[i];
        for(int b=0;b<8;b++){
            uint32_t m = -(crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & m);
        }
    }
    return ~crc;
}

// ===== Формат заголовка во Flash =====
typedef struct {
    uint32_t magic;    // 'P''L''C''G' (0x504C4347)
    uint16_t version;  // 1
    uint16_t rsv;      // 0
    uint32_t len;      // длина CBOR-графа (байт)
    uint32_t crc32;    // CRC32 от CBOR-графа
    uint32_t stampMs;  // now_ms() при сохранении (опционально)
} PlcPersistHeader;

#define PLC_PERSIST_MAGIC 0x504C4347u
#define PLC_PERSIST_VER   1u

// ===== ВСПОМОГАТЕЛЬНОЕ: дамп N байт =====
static void _hexdump(const void* data, size_t len, size_t max){
    const uint8_t* p = (const uint8_t*)data;
    size_t n = (len < max) ? len : max;
    for (size_t i=0;i<n;i++){
        if ((i & 0x0F)==0) printf("\r\n%08X: ", (unsigned int)i);
        printf("%02X ", p[i]);
    }
    printf("\r\n");
}

static bool encode_param_float_fallback(CborEncoder* map, const char* key, float v){
    // 1) Пытаемся как нормальный CBOR float (если FPU/ABI ок)
#if defined(__FPU_PRESENT) && (__FPU_PRESENT==1) && defined(__FPU_USED) && (__FPU_USED==1)
    if (!_ok(cbor_encode_text_stringz(map, key))) return false;
    if (!_ok(cbor_encode_float(map, v))) return false;
    return true;
#else
    // 2) Без FPU — пишем сырые биты в paramFloatBits (uint32)
    uint32_t bits; memcpy(&bits, &v, sizeof(bits));
    if (!_ok(cbor_encode_text_stringz(map, "paramFloatBits"))) return false;
    if (!_ok(cbor_encode_uint(map, (uint64_t)bits))) return false;
    return true;
#endif
}


// ===== Кодирование текущего active-graph в CBOR =====
static bool encode_graph_cbor(uint8_t* buf, size_t cap, size_t* outLen){
    // Снимок под критической секцией
    PlcGraph g;
    bool valid;
    taskENTER_CRITICAL();
    valid = g_activeGraphValid;
    if (valid) g = g_activeGraph;
    taskEXIT_CRITICAL();

    if (!valid) {
        LOGT(tag, "encode_graph_cbor: g_activeGraphValid = false");
        return false;
    }

    LOGT(tag, "encode_graph_cbor: cycleMs=%lu nodeCount=%u cap=%lu",
         (unsigned long)g.cycleMs, (unsigned)g.nodeCount, (unsigned long)cap);

    CborEncoder root, map, arr, nmap;
    cbor_encoder_init(&root, buf, cap, 0);

    if (!_ok(cbor_encoder_create_map(&root, &map, CborIndefiniteLength))) return false;

    // "cycleMs": uint
    if (!_ok(cbor_encode_text_stringz(&map, "cycleMs"))) return false;
    if (!_ok(cbor_encode_uint(&map, g.cycleMs))) return false;

    // "nodes": [...]
    if (!_ok(cbor_encode_text_stringz(&map, "nodes"))) return false;
    if (!_ok(cbor_encoder_create_array(&map, &arr, g.nodeCount))) return false;

    for (uint16_t i=0;i<g.nodeCount;i++){
        PlcNode* n = &g.nodes[i];
        if (!_ok(cbor_encoder_create_map(&arr, &nmap, CborIndefiniteLength))) return false;

        if (!_ok(cbor_encode_text_stringz(&nmap, "id")))        return false; if (!_ok(cbor_encode_uint(&nmap, n->id))) return false;
        if (!_ok(cbor_encode_text_stringz(&nmap, "type")))      return false; if (!_ok(cbor_encode_uint(&nmap, n->type))) return false;
        if (!_ok(cbor_encode_text_stringz(&nmap, "valueType"))) return false; if (!_ok(cbor_encode_uint(&nmap, n->valueType))) return false;
        if (!_ok(cbor_encode_text_stringz(&nmap, "inA")))       return false; if (!_ok(cbor_encode_int(&nmap, n->inA))) return false;
        if (!_ok(cbor_encode_text_stringz(&nmap, "inB")))       return false; if (!_ok(cbor_encode_int(&nmap, n->inB))) return false;
        if (!_ok(cbor_encode_text_stringz(&nmap, "paramInt")))  return false; if (!_ok(cbor_encode_int(&nmap, n->paramInt))) return false;
//        if (!_ok(cbor_encode_text_stringz(&nmap, "paramFloat")))return false; if (!_ok(cbor_encode_float(&nmap, n->paramFloat))) return false;
        if (!encode_param_float_fallback(&nmap, "paramFloat", n->paramFloat)) return false;
        if (!_ok(cbor_encode_text_stringz(&nmap, "paramMs")))   return false; if (!_ok(cbor_encode_uint(&nmap, n->paramMs))) return false;
        if (!_ok(cbor_encode_text_stringz(&nmap, "flags")))     return false; if (!_ok(cbor_encode_uint(&nmap, n->flags))) return false;

        if (!_ok(cbor_encoder_close_container(&arr, &nmap))) return false;
    }

    if (!_ok(cbor_encoder_close_container(&map, &arr))) return false;
    if (!_ok(cbor_encoder_close_container(&root, &map))) return false;

    *outLen = cbor_encoder_get_buffer_size(&root, buf);
    LOGT(tag, "encode_graph_cbor: outLen=%lu", (unsigned long)*outLen);
    return true;
}

// ===== Декодирование CBOR → PlcGraph =====
extern bool decode_graph_from_cbor(const uint8_t* buf, size_t len, PlcGraph* outG);

// ===== Низкоуровневые операции с FLASH =====
static bool flash_erase_sector(void){
    LOGT(tag, "flash_erase_sector: sector=%lu addr=0x%08lX size=%lu",
         (unsigned long)PLC_FLASH_SECTOR_ID,
         (unsigned long)PLC_FLASH_SECTOR_ADDR,
         (unsigned long)PLC_FLASH_SECTOR_SIZE);

    _log_optcr();

    HAL_FLASH_Unlock();

    // Сбросим возможные флаги ошибок перед операцией
    __HAL_FLASH_CLEAR_FLAG(FLASH_SR_EOP | FLASH_SR_WRPERR |
                           FLASH_SR_PGAERR | FLASH_SR_PGPERR | FLASH_SR_PGSERR
#if defined(FLASH_SR_RDERR)
                                   | FLASH_SR_RDERR
#endif
    );

    FLASH_EraseInitTypeDef ei = {0};
    ei.TypeErase    = TYPEERASE_SECTORS;
    ei.Sector       = PLC_FLASH_SECTOR_ID;
    ei.NbSectors    = 1;
    ei.VoltageRange = VOLTAGE_RANGE_3; // 2.7–3.6V

    uint32_t err = 0;
    HAL_StatusTypeDef st = HAL_FLASHEx_Erase(&ei, &err);

    uint32_t sr = FLASH->SR;
    _log_flash_sr(sr);

    HAL_FLASH_Lock();

    if (st != HAL_OK){
        LOGT(tag, "flash_erase_sector: FAIL st=%ld err_index=%lu", (long)st, (unsigned long)err);
        return false;
    }

    // Быстрая проверка "чистоты" (не весь сектор, а выборочно)
    bool ff = _probe_ff(PLC_FLASH_SECTOR_ADDR, 128);
    LOGT(tag, "flash_erase_sector: OK, first128=%s", ff ? "FF" : "NON-FF");
    if (!ff) {
        LOGT(tag, "flash_erase_sector: dump first 64B:");
        _hexdump_at(PLC_FLASH_SECTOR_ADDR, 64, 64);
        return false;
    }
    return true;
}


static bool flash_write_aligned(uint32_t addr, const uint8_t* data, size_t len){
    if ((addr & 0x3u) != 0){
        LOGT(tag, "flash_write_aligned: UNALIGNED addr=0x%08lX", (unsigned long)addr);
        return false;
    }
    if (!_addr_in_sector(addr, len)) {
        LOGT(tag, "flash_write_aligned: OUT-OF-BOUNDS addr=0x%08lX len=%lu (sec 0x%08lX..0x%08lX)",
             (unsigned long)addr, (unsigned long)len,
             (unsigned long)PLC_FLASH_SECTOR_ADDR,
             (unsigned long)(PLC_FLASH_SECTOR_ADDR + PLC_FLASH_SECTOR_SIZE));
        return false;
    }

    size_t aligned = (len + 3u) & ~3u;
    LOGT(tag, "flash_write_aligned: start=0x%08lX len=%lu aligned=%lu",
         (unsigned long)addr, (unsigned long)len, (unsigned long)aligned);

    HAL_FLASH_Unlock();

    // Очистим флаги перед записью
    __HAL_FLASH_CLEAR_FLAG(FLASH_SR_EOP | FLASH_SR_WRPERR |
                           FLASH_SR_PGAERR | FLASH_SR_PGPERR | FLASH_SR_PGSERR
#if defined(FLASH_SR_RDERR)
                                   | FLASH_SR_RDERR
#endif
    );

    size_t i=0;
    uint32_t baseAddr = addr;
    while (i < aligned) {
        uint32_t word = 0xFFFFFFFFu;
        for (int b=0;b<4;b++){
            size_t k = i + b;
            ((uint8_t*)&word)[b] = (k < len) ? data[k] : 0xFF;
        }

        HAL_StatusTypeDef st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, word);
        if (st != HAL_OK){
            _log_flash_sr(FLASH->SR);
            LOGT(tag, "flash_write_aligned: PROG_FAIL off=%lu addr=0x%08lX st=%ld",
                 (unsigned long)i, (unsigned long)addr, (long)st);
            HAL_FLASH_Lock();
            return false;
        }

        // Быстрая сверка записанного слова
        uint32_t rd = *(volatile uint32_t*)addr;
        if (rd != word){
            LOGT(tag, "flash_write_aligned: VERIFY_WORD_FAIL off=%lu addr=0x%08lX wr=0x%08lX rd=0x%08lX",
                 (unsigned long)i, (unsigned long)addr,
                 (unsigned long)word, (unsigned long)rd);
            HAL_FLASH_Lock();
            return false;
        }

        addr += 4; i += 4;

        // Ненавязчивый прогресс раз в 256 байт
        if ( (i & 0xFF) == 0 ) LOGT(tag, "flash_write_aligned: progress %lu/%lu", (unsigned long)i, (unsigned long)aligned);
    }

    HAL_FLASH_Lock();
    LOGT(tag, "flash_write_aligned: OK");

    // Итоговая точечная проверка начала/конца
    LOGT(tag, "flash_write_aligned: verify head/tail");
    _hexdump_at(baseAddr, (len>32?32:len), 32);
    if (len > 32) _hexdump_at(baseAddr + len - (len>32?32:len), (len>32?32:len), 32);

    return true;
}


// ===== Публичные API =====
bool plc_persist_save_active(void){
    static uint8_t buf[32 + 6144];

    size_t cborLen = 0;
    if (!encode_graph_cbor(buf + sizeof(PlcPersistHeader),
                           sizeof(buf) - sizeof(PlcPersistHeader),
                           &cborLen)) {
        LOGT(tag, "save_active: encode_graph_cbor -> FAIL");
        return false;
    }

    PlcPersistHeader hdr = {0};
    hdr.magic   = PLC_PERSIST_MAGIC;
    hdr.version = PLC_PERSIST_VER;
    hdr.len     = (uint32_t)cborLen;
    hdr.crc32   = crc32_update(0, buf + sizeof(PlcPersistHeader), cborLen);
    hdr.stampMs = now_ms();

    memcpy(buf, &hdr, sizeof(hdr));
    size_t totalLen = sizeof(hdr) + cborLen;

    LOGT(tag, "save_active: hdr{magic=0x%08lX ver=%u len=%lu crc=0x%08lX stamp=%lu} totalLen=%lu",
         (unsigned long)hdr.magic, (unsigned)hdr.version,
         (unsigned long)hdr.len, (unsigned long)hdr.crc32, (unsigned long)hdr.stampMs,
         (unsigned long)totalLen);

    LOGT(tag, "save_active: sector check: base=0x%08lX size=%lu app_end_hint=FLASH END UNKNOWN");
    // ВАЖНО: проверь, что PLC_FLASH_SECTOR_ADDR — вне области прошивки по линкер-скрипту!

    if (!flash_erase_sector()){
        LOGT(tag, "save_active: erase -> FAIL");
        return false;
    }

    if (!flash_write_aligned(PLC_FLASH_SECTOR_ADDR, buf, totalLen)){
        LOGT(tag, "save_active: write -> FAIL");
        return false;
    }

    const PlcPersistHeader* vh = (const PlcPersistHeader*)PLC_FLASH_SECTOR_ADDR;
    uint32_t vcrc = crc32_update(0,
                                 (const uint8_t*)(PLC_FLASH_SECTOR_ADDR + sizeof(PlcPersistHeader)),
                                 vh->len);

    LOGT(tag, "save_active: verify header{magic=0x%08lX ver=%u len=%lu crc=0x%08lX} vcrc=0x%08lX",
         (unsigned long)vh->magic, (unsigned)vh->version, (unsigned long)vh->len,
         (unsigned long)vh->crc32, (unsigned long)vcrc);

    if (vh->magic != PLC_PERSIST_MAGIC || vh->version != PLC_PERSIST_VER || vcrc != vh->crc32){
        LOGT(tag, "save_active: VERIFY FAIL -> dump header area:");
        _hexdump_at(PLC_FLASH_SECTOR_ADDR, sizeof(PlcPersistHeader)+32, sizeof(PlcPersistHeader)+32);
        return false;
    }

    LOGT(tag, "save_active: OK");
    return true;
}


bool plc_persist_load_active(void)
{
    // 1) Прочитать заголовок
    const PlcPersistHeader* hdr = (const PlcPersistHeader*)PLC_FLASH_SECTOR_ADDR;
    LOGT(tag, "load_active: hdr{magic=0x%08lX ver=%u len=%lu crc=0x%08lX stamp=%lu}",
         (unsigned long)hdr->magic, (unsigned)hdr->version,
         (unsigned long)hdr->len, (unsigned long)hdr->crc32, (unsigned long)hdr->stampMs);

    if (hdr->magic != PLC_PERSIST_MAGIC || hdr->version != PLC_PERSIST_VER) {
        LOGT(tag, "load_active: BAD MAGIC/VER");
        return false;
    }
    if (hdr->len == 0 || hdr->len > PLC_FLASH_SECTOR_SIZE - sizeof(PlcPersistHeader)) {
        LOGT(tag, "load_active: BAD LEN %lu (sector_size=%lu)",
             (unsigned long)hdr->len, (unsigned long)PLC_FLASH_SECTOR_SIZE);
        return false;
    }

    const uint8_t* cbor = (const uint8_t*)(PLC_FLASH_SECTOR_ADDR + sizeof(PlcPersistHeader));
    uint32_t crcCalc = crc32_update(0, cbor, hdr->len);
    LOGT(tag, "load_active: crcCalc=0x%08lX (expect=0x%08lX)", (unsigned long)crcCalc, (unsigned long)hdr->crc32);
    if (crcCalc != hdr->crc32) {
        LOGT(tag, "load_active: CRC MISMATCH");
        return false;
    }

    // 2) Декодировать граф
    PlcGraph tmp;
    bool decOk = decode_graph_from_cbor(cbor, hdr->len, &tmp);
    LOGT(tag, "load_active: decode_graph_from_cbor -> %s", decOk ? "OK" : "FAIL");
    if (!decOk) return false;

    // 3) Проверка и активация
    bool upOk = plc_upload_graph(&tmp);
    LOGT(tag, "load_active: plc_upload_graph -> %s", upOk ? "OK" : "FAIL");
    if (!upOk) return false;

    plc_request_activate_graph();
    LOGT(tag, "load_active: activated");
    return true;
}

// ===== Доп: быстрая проверка содержимого сектора (по требованию) =====
void plc_persist_debug_dump_header(void){
    const PlcPersistHeader* h = (const PlcPersistHeader*)PLC_FLASH_SECTOR_ADDR;
    LOGT(tag, "HDR dump: magic=0x%08lX ver=%u len=%lu crc=0x%08lX stamp=%lu",
         (unsigned long)h->magic, (unsigned)h->version, (unsigned long)h->len,
         (unsigned long)h->crc32, (unsigned long)h->stampMs);
    const uint8_t* cbor = (const uint8_t*)(PLC_FLASH_SECTOR_ADDR + sizeof(PlcPersistHeader));
    _hexdump(cbor, h->len, 96);
}
