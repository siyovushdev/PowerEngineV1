#include "cbor_msg.h"
#include "rx_to_idle_dma.h"
#include "adc.h"
#include "plc.h"
#include <string.h>

static bool get_int32(const CborValue* val, int32_t* out)
{
    if (!cbor_value_is_integer(val)) return false;
    int64_t tmp;
    CborError err = cbor_value_get_int64(val, &tmp);
    if (err != CborNoError) return false;
    *out = (int32_t)tmp;
    return true;
}

static bool get_uint32(const CborValue* val, uint32_t* out)
{
    if (!cbor_value_is_unsigned_integer(val) &&
        !cbor_value_is_integer(val)) return false;
    uint64_t tmp;
    CborError err = cbor_value_get_uint64(val, &tmp);
    if (err != CborNoError) return false;
    *out = (uint32_t)tmp;
    return true;
}

static bool get_float(const CborValue* val, float* out)
{
    if (!(cbor_value_is_half_float(val) ||
          cbor_value_is_float(val) ||
          cbor_value_is_double(val))) {
        return false;
    }
    float tmp;
    CborError err = cbor_value_get_float(val, &tmp); // get_double умеет и half/float
    if (err != CborNoError) return false;
    *out = (float)tmp;
    return true;
}

static CborError decode_single_node(CborValue *nodeMapIt, PlcNode *outNode)
{
    // Инициируем дефолты (на случай отсутствующих ключей)
    outNode->id         = 0;
    outNode->type       = 0;
    outNode->valueType  = 0;
    outNode->inA        = -1;
    outNode->inB        = -1;
    outNode->paramInt   = 0;
    outNode->paramFloat = 0.0f;
    outNode->paramMs    = 0;
    outNode->flags      = 0;

    if (!cbor_value_is_map(nodeMapIt)) {
        return CborErrorIllegalType;
    }

    CborValue val;

    // id
    cbor_value_map_find_value(nodeMapIt, "id", &val);
    if (cbor_value_is_valid(&val)) {
        int32_t tmp;
        if (!get_int32(&val, &tmp)) return CborErrorIllegalType;
        outNode->id = tmp;
    }

    // type
    cbor_value_map_find_value(nodeMapIt, "type", &val);
    if (cbor_value_is_valid(&val)) {
        int32_t tmp;
        if (!get_int32(&val, &tmp)) return CborErrorIllegalType;
        outNode->type = (uint8_t)tmp;
    }

    // valueType
    cbor_value_map_find_value(nodeMapIt, "valueType", &val);
    if (cbor_value_is_valid(&val)) {
        int32_t tmp;
        if (!get_int32(&val, &tmp)) return CborErrorIllegalType;
        outNode->valueType = (uint8_t)tmp;
    }

    // inA
    cbor_value_map_find_value(nodeMapIt, "inA", &val);
    if (cbor_value_is_valid(&val)) {
        int32_t tmp;
        if (!get_int32(&val, &tmp)) return CborErrorIllegalType;
        outNode->inA = (int16_t)tmp;
    }

    // inB
    cbor_value_map_find_value(nodeMapIt, "inB", &val);
    if (cbor_value_is_valid(&val)) {
        int32_t tmp;
        if (!get_int32(&val, &tmp)) return CborErrorIllegalType;
        outNode->inB = (int16_t)tmp;
    }

    // paramInt
    cbor_value_map_find_value(nodeMapIt, "paramInt", &val);
    if (cbor_value_is_valid(&val)) {
        int32_t tmp;
        if (!get_int32(&val, &tmp)) return CborErrorIllegalType;
        outNode->paramInt = tmp;
    }

    // paramFloat
    cbor_value_map_find_value(nodeMapIt, "paramFloat", &val);
    if (cbor_value_is_valid(&val)) {
        float tmpf;
        if (!get_float(&val, &tmpf)) return CborErrorIllegalType;
        outNode->paramFloat = tmpf;
    }

    // paramMs
    cbor_value_map_find_value(nodeMapIt, "paramMs", &val);
    if (cbor_value_is_valid(&val)) {
        uint32_t tmpu;
        // paramMs может быть как signed (3000) так и unsigned
        // используем get_uint32 чтобы не ругался
        if (!get_uint32(&val, &tmpu)) return CborErrorIllegalType;
        outNode->paramMs = tmpu;
    }

    // flags
    cbor_value_map_find_value(nodeMapIt, "flags", &val);
    if (cbor_value_is_valid(&val)) {
        uint32_t tmpu;
        if (!get_uint32(&val, &tmpu)) return CborErrorIllegalType;
        outNode->flags = tmpu;
    }

    return CborNoError;
}

static CborError decode_graph(CborValue *graphMapIt, PlcGraph *outGraph)
{
    if (!cbor_value_is_map(graphMapIt)) {
        return CborErrorIllegalType;
    }

    // defaults
    outGraph->cycleMs   = 10;
    outGraph->nodeCount = 0;

    CborValue val;

    // cycleMs
    cbor_value_map_find_value(graphMapIt, "cycleMs", &val);
    if (cbor_value_is_valid(&val)) {
        uint32_t cyc;
        if (!get_uint32(&val, &cyc)) return CborErrorIllegalType;
        outGraph->cycleMs = cyc;
    }

    // nodes[]
    cbor_value_map_find_value(graphMapIt, "nodes", &val);
    if (!cbor_value_is_valid(&val)) {
        // nodes must exist
        return CborErrorUnknownType; // или свой код ошибки
    }
    if (!cbor_value_is_array(&val)) {
        return CborErrorIllegalType;
    }

    // Итерируем массив узлов
    CborValue arrIt;
    CborError err = cbor_value_enter_container(&val, &arrIt);
    if (err != CborNoError) return err;

    uint16_t count = 0;
    while (!cbor_value_at_end(&arrIt)) {
        if (count >= PLC_MAX_NODES) {
            return CborErrorOutOfMemory; // слишком много узлов
        }

        if (!cbor_value_is_map(&arrIt)) {
            return CborErrorIllegalType;
        }

        err = decode_single_node(&arrIt, &outGraph->nodes[count]);
        if (err != CborNoError) {
            return err;
        }

        // перешагиваем к следующему элементу массива
        err = cbor_value_advance(&arrIt);
        if (err != CborNoError) {
            return err;
        }

        count++;
    }

    outGraph->nodeCount = count;

    // Выйти из массива
    err = cbor_value_leave_container(&val, &arrIt);
    return err;
}

// Эта функция вызывается после того как мы приняли и проверили кадр (CRC и т.д.)
bool parse_cmd_upload_graph(const uint8_t *buf, size_t len)
{
    CborParser parser;
    CborValue root;
    CborError err;

    err = cbor_parser_init(buf, len, 0, &parser, &root);
    if (err != CborNoError) {
        return false;
    }

    // Ждём корневую map
    if (!cbor_value_is_map(&root)) {
        return false;
    }

    // Проверяем cmd
    CborValue valCmd;
    cbor_value_map_find_value(&root, "cmd", &valCmd);
    if (!cbor_value_is_text_string(&valCmd)) {
        return false;
    }

    char cmdBuf[32];
    size_t cmdLen = sizeof(cmdBuf) - 1;
    err = cbor_value_copy_text_string(&valCmd, cmdBuf, &cmdLen, NULL);
    if (err != CborNoError) {
        return false;
    }
    cmdBuf[cmdLen] = '\0';

    if (strcmp(cmdBuf, "upload_graph") != 0) {
        // не наша команда
        return false;
    }

    // Достаём "graph"
    CborValue valGraph;
    cbor_value_map_find_value(&root, "graph", &valGraph);
    if (!cbor_value_is_map(&valGraph)) {
        return false;
    }

    PlcGraph tmp;
    err = decode_graph(&valGraph, &tmp);
    if (err != CborNoError) {
        return false;
    }

    // Если дошли сюда — граф корректен.
    // Кладём во staging и отмечаем флаги.
    g_stagingGraph       = tmp;
    g_stagingGraphValid  = true;
    g_needSwapGraph      = true;

    return true;
}

/* Точка входа из транспорта */
void on_cbor_frame_received(const uint8_t *buf, size_t len) {
    parse_cmd_upload_graph(buf, len);
}



