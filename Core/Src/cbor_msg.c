#include "cbor_msg.h"


#define LOGF(fmt, ...)  printf("CBOR %s:%d " fmt "\r\n", __FUNCTION__, __LINE__, ##__VA_ARGS__)

static const char *cbor_type_name(CborType t) {
    switch (t) {
        case CborIntegerType:
            return "int";
        case CborByteStringType:
            return "bstr";
        case CborTextStringType:
            return "tstr";
        case CborArrayType:
            return "array";
        case CborMapType:
            return "map";
        case CborTagType:
            return "tag";
        case CborSimpleType:
            return "simple/null/undef/bool";
        case CborHalfFloatType:
            return "half";
        case CborFloatType:
            return "float";
        case CborDoubleType:
            return "double";
        case CborInvalidType:
        default:
            return "invalid";
    }
}

static inline bool _ok(CborError e){ return e==CborNoError; }

void send_status_reply(CborReply *out)
{
    // Буфер ответа (подбери при необходимости)
    CborEncoder root, map, arr, nmap;

    // Снимок для консистентности
    bool     active;
    uint32_t cycle;
    uint16_t cnt;

    taskENTER_CRITICAL();
    active = g_activeGraphValid;
    cycle  = active ? g_activeGraph.cycleMs : 0;
    cnt    = active ? g_activeGraph.nodeCount : 0;
    taskEXIT_CRITICAL();

    cbor_encoder_init(&root, out->data, sizeof(out->data), 0);
    if(!_ok(cbor_encoder_create_map(&root, &map, CborIndefiniteLength))) goto send;

    // "cmd": "status"
    if(!_ok(cbor_encode_text_stringz(&map, "cmd"))) goto send;
    if(!_ok(cbor_encode_text_stringz(&map, "status"))) goto send;

    // uptimeMs
    if(!_ok(cbor_encode_text_stringz(&map, "uptimeMs"))) goto send;
    if(!_ok(cbor_encode_uint(&map, now_ms()))) goto send;

    // active
    if(!_ok(cbor_encode_text_stringz(&map, "active"))) goto send;
    if(!_ok(cbor_encode_boolean(&map, active))) goto send;

    // cycleMs
    if(!_ok(cbor_encode_text_stringz(&map, "cycleMs"))) goto send;
    if(!_ok(cbor_encode_uint(&map, cycle))) goto send;

    // nodesCount
    if(!_ok(cbor_encode_text_stringz(&map, "nodesCount"))) goto send;
    if(!_ok(cbor_encode_uint(&map, cnt))) goto send;

    // nodesState: массив узлов
    if(!_ok(cbor_encode_text_stringz(&map, "nodesState"))) goto send;
    if(!_ok(cbor_encoder_create_array(&map, &arr, cnt))) goto send;

    if (active) {
        taskENTER_CRITICAL();
        for (uint16_t i = 0; i < g_activeGraph.nodeCount; i++) {
            PlcNode* n = &g_activeGraph.nodes[i];

            if(!_ok(cbor_encoder_create_map(&arr, &nmap, CborIndefiniteLength))) { taskEXIT_CRITICAL(); goto send; }

            // id
            if(!_ok(cbor_encode_text_stringz(&nmap, "id"))) goto ns_close;
            if(!_ok(cbor_encode_uint(&nmap, n->id))) goto ns_close;

            // type
            if(!_ok(cbor_encode_text_stringz(&nmap, "type"))) goto ns_close;
            if(!_ok(cbor_encode_uint(&nmap, n->type))) goto ns_close;

            // valueType
            if(!_ok(cbor_encode_text_stringz(&nmap, "valueType"))) goto ns_close;
            if(!_ok(cbor_encode_uint(&nmap, n->valueType))) goto ns_close;

            // out*
            if (n->valueType == PLC_VAL_BOOL) {
                if(!_ok(cbor_encode_text_stringz(&nmap, "outBool"))) goto ns_close;
                if(!_ok(cbor_encode_boolean(&nmap, n->out.b))) goto ns_close;
            } else if (n->valueType == PLC_VAL_INT) {
                if(!_ok(cbor_encode_text_stringz(&nmap, "outInt"))) goto ns_close;
                if(!_ok(cbor_encode_int(&nmap, n->out.i))) goto ns_close;
            } else { // FLOAT
                if(!_ok(cbor_encode_text_stringz(&nmap, "outFloat"))) goto ns_close;
                if(!_ok(cbor_encode_float(&nmap, n->out.f))) goto ns_close;
            }

            // спец-поля для таймеров/форса (по возможности)
            if (n->type == PLC_NODE_TON) {
                if(!_ok(cbor_encode_text_stringz(&nmap, "tonMs"))) goto ns_close;
                if(!_ok(cbor_encode_uint(&nmap, n->ton_accum_ms))) goto ns_close;
            }
            if (n->type == PLC_NODE_TOFF) {
                if(!_ok(cbor_encode_text_stringz(&nmap, "toffLeftMs"))) goto ns_close;
                if(!_ok(cbor_encode_uint(&nmap, n->toff_left_ms))) goto ns_close;
            }
            if (n->flags & PLC_NODE_FLAG_FORCE) {
                if(!_ok(cbor_encode_text_stringz(&nmap, "force"))) goto ns_close;
                if(!_ok(cbor_encode_boolean(&nmap, n->force_en))) goto ns_close;

                if(!_ok(cbor_encode_text_stringz(&nmap, "forceLeftMs"))) goto ns_close;
                if(!_ok(cbor_encode_uint(&nmap, n->force_left_ms))) goto ns_close;
            }

            if(!_ok(cbor_encoder_close_container(&arr, &nmap))) { taskEXIT_CRITICAL(); goto send; }
            continue;

            ns_close:
            (void)cbor_encoder_close_container(&arr, &nmap);
            taskEXIT_CRITICAL();
            goto send;
        }
        taskEXIT_CRITICAL();
    }

    if(!_ok(cbor_encoder_close_container(&map, &arr))) goto send;

    // закрыть корневую map
    if(!_ok(cbor_encoder_close_container(&root, &map))) goto send;

    send: ;
    out->len = (uint16_t)cbor_encoder_get_buffer_size(&root, out->data);
}

static void send_upload_graph_ack(bool ok, CborReply *out)
{
    if (!out) return; out->len = 0;

    CborEncoder root, map;

    uint16_t cnt = 0; uint32_t cycle = 0;
    if (ok) {
        taskENTER_CRITICAL();
        cnt   = g_stagingGraphValid ? g_stagingGraph.nodeCount : 0;
        cycle = g_stagingGraphValid ? g_stagingGraph.cycleMs   : 0;
        taskEXIT_CRITICAL();
    }

    cbor_encoder_init(&root, out->data, sizeof(out->data), 0);
    if(!_ok(cbor_encoder_create_map(&root, &map, CborIndefiniteLength))) goto out;

    // cmd
    if(!_ok(cbor_encode_text_stringz(&map, "cmd"))) goto out;
    if(!_ok(cbor_encode_text_stringz(&map, "upload_graph_ack"))) goto out;

    // ok
    if(!_ok(cbor_encode_text_stringz(&map, "ok"))) goto out;
    if(!_ok(cbor_encode_boolean(&map, ok))) goto out;

    if (ok) {
        if(!_ok(cbor_encode_text_stringz(&map, "stagingNodeCount"))) goto out;
        if(!_ok(cbor_encode_uint(&map, cnt))) goto out;

        if(!_ok(cbor_encode_text_stringz(&map, "stagingCycleMs"))) goto out;
        if(!_ok(cbor_encode_uint(&map, cycle))) goto out;
    }

    (void)cbor_encoder_close_container(&root, &map);
    out:
    out->len = (uint16_t)cbor_encoder_get_buffer_size(&root, out->data);
}

static void send_simple_ack(const char* cmdName, bool ok, CborReply *out)
{
    CborEncoder root, map;

    cbor_encoder_init(&root, out->data, sizeof(out->data), 0);
    if(!_ok(cbor_encoder_create_map(&root, &map, CborIndefiniteLength))) goto out;

    if(!_ok(cbor_encode_text_stringz(&map, "cmd"))) goto out;
    if(!_ok(cbor_encode_text_stringz(&map, cmdName))) goto out;

    if(!_ok(cbor_encode_text_stringz(&map, "ok"))) goto out;
    if(!_ok(cbor_encode_boolean(&map, ok))) goto out;

    (void)cbor_encoder_close_container(&root, &map);
    out:
    out->len = (uint16_t)cbor_encoder_get_buffer_size(&root, out->data);
}

static void send_activate_graph_ack(bool ok, CborReply *out)  { send_simple_ack("activate_graph_ack", ok, out); }
static void send_force_output_ack(bool ok, CborReply *out)    { send_simple_ack("force_output_ack", ok, out); }

static bool get_int32(const CborValue *val, int32_t *out) {
    if (!cbor_value_is_integer(val)) return false;
    int64_t tmp;
    CborError err = cbor_value_get_int64(val, &tmp);
    if (err != CborNoError) return false;
    *out = (int32_t) tmp;
    return true;
}

static bool get_uint32(const CborValue *val, uint32_t *out) {
    if (!cbor_value_is_unsigned_integer(val) &&
        !cbor_value_is_integer(val))
        return false;
    uint64_t tmp;
    CborError err = cbor_value_get_uint64(val, &tmp);
    if (err != CborNoError) return false;
    *out = (uint32_t) tmp;
    return true;
}

static bool get_float(const CborValue *val, float *out) {
    if (!(cbor_value_is_half_float(val) ||
          cbor_value_is_float(val) ||
          cbor_value_is_double(val))) {
        return false;
    }
    float tmp;
    CborError err = cbor_value_get_float(val, &tmp); // get_double умеет и half/float
    if (err != CborNoError) return false;
    *out = (float) tmp;
    return true;
}

static bool get_float_any(const CborValue *val, float *out) {
    const CborType t = cbor_value_get_type(val);

    if (t == CborHalfFloatType) {
        float f;
        CborError e = cbor_value_get_half_float(val, &f);  // <-- ДЛЯ HALF
        if (e != CborNoError) return false;
        *out = f;
        return true;
    }

    if (t == CborFloatType) {
        float f;
        CborError e = cbor_value_get_float(val, &f);       // <-- ДЛЯ FLOAT
        if (e != CborNoError) return false;
        *out = f;
        return true;
    }

    if (t == CborDoubleType) {
        double d;
        CborError e = cbor_value_get_double(val, &d);      // <-- ДЛЯ DOUBLE
        if (e != CborNoError) return false;
        *out = (float)d;
        return true;
    }

    if (t == CborIntegerType) {
        int64_t v;
        CborError e = cbor_value_get_int64(val, &v);
        if (e != CborNoError) return false;
        *out = (float)v;
        return true;
    }

    return false;
}


// helper
static inline bool map_find(CborValue *mapCont, const char *key, CborValue *out) {
    CborValue it = *mapCont;                     // КОПИЯ!
    return cbor_value_map_find_value(&it, key, out) == CborNoError
           && cbor_value_is_valid(out);
}

static CborError decode_single_node(CborValue *nodeMapIt, PlcNode *outNode) {
    // Инициируем дефолты (на случай отсутствующих ключей)
    outNode->id = 0;
    outNode->type = 0;
    outNode->valueType = 0;
    outNode->inA = -1;
    outNode->inB = -1;
    outNode->paramInt = 0;
    outNode->paramFloat = 0.0f;
    outNode->paramMs = 0;
    outNode->flags = 0;

    if (!cbor_value_is_map(nodeMapIt)) {
        LOGF("node is not map, type=%s", cbor_type_name(cbor_value_get_type(nodeMapIt)));
        return CborErrorIllegalType;
    }

    CborValue val;

    // id
    if (map_find(nodeMapIt, "id", &val)) {
        LOGF("id type=%s", cbor_type_name(cbor_value_get_type(&val)));
        int32_t tmp;
        if (!get_int32(&val, &tmp)) {
            LOGF("id parse error");
            return CborErrorIllegalType;
        }
        outNode->id = tmp;
    } else {
        LOGF("id not present");
    }


    // type
    if (map_find(nodeMapIt, "type", &val)) {
        LOGF("type type=%s", cbor_type_name(cbor_value_get_type(&val)));
        int32_t tmp;
        if (!get_int32(&val, &tmp)) {
            LOGF("type parse error");
            return CborErrorIllegalType;
        }
        outNode->type = tmp;
    } else {
        LOGF("type not present");
    }

    // valueType
    if (map_find(nodeMapIt, "valueType", &val)) {
        LOGF("valueType type=%s", cbor_type_name(cbor_value_get_type(&val)));
        int32_t tmp;
        if (!get_int32(&val, &tmp)) {
            LOGF("valueType parse error");
            return CborErrorIllegalType;
        }
        outNode->valueType = tmp;
    } else {
        LOGF("valueType not present");
    }

    // inA
    if (map_find(nodeMapIt, "inA", &val)) {
        LOGF("inA type=%s", cbor_type_name(cbor_value_get_type(&val)));
        int32_t tmp;
        if (!get_int32(&val, &tmp)) {
            LOGF("inA parse error");
            return CborErrorIllegalType;
        }
        outNode->inA = (int16_t) tmp;
    } else {
        LOGF("inA not present");
    }

    // inB
    if (map_find(nodeMapIt, "inB", &val)) {
        LOGF("inB type=%s", cbor_type_name(cbor_value_get_type(&val)));
        int32_t tmp;
        if (!get_int32(&val, &tmp)) {
            LOGF("inB parse error");
            return CborErrorIllegalType;
        }
        outNode->inB = (int16_t) tmp;
    } else {
        LOGF("inB not present");
    }

    // paramInt
    if (map_find(nodeMapIt, "paramInt", &val)) {
        LOGF("paramInt type=%s", cbor_type_name(cbor_value_get_type(&val)));
        int32_t tmp;
        if (!get_int32(&val, &tmp)) {
            LOGF("paramInt parse error");
            return CborErrorIllegalType;
        }
        outNode->paramInt =  tmp;
    } else {
        LOGF("paramInt not present");
    }

    // paramFloat
    if (map_find(nodeMapIt, "paramFloat", &val)) {
        LOGF("paramFloat type=%s", cbor_type_name(cbor_value_get_type(&val)));
        float tmpf;
        if (!get_float_any(&val, &tmpf)) {
            LOGF("paramFloat parse error");
            return CborErrorIllegalType;
        }
        LOGF("paramFloat=%.6f", tmpf);
        outNode->paramFloat = tmpf;
    } else {
        LOGF("paramFloat not present");
    }

    // paramMs
    if (map_find(nodeMapIt, "paramMs", &val)) {
        LOGF("paramMs type=%s", cbor_type_name(cbor_value_get_type(&val)));
        uint32_t tmpu;
        if (!get_uint32(&val, &tmpu)) {
            LOGF("paramMs parse error");
            return CborErrorIllegalType;
        }
        outNode->paramMs = tmpu;
    } else {
        LOGF("paramMs not present");
    }

    // flags
    if (map_find(nodeMapIt, "flags", &val)) {
        LOGF("flags type=%s", cbor_type_name(cbor_value_get_type(&val)));
        uint32_t tmpu;
        if (!get_uint32(&val, &tmpu)) {
            LOGF("flags parse error");
            return CborErrorIllegalType;
        }
        outNode->flags = tmpu;
    } else {
        LOGF("flags not present");
    }

    return CborNoError;
}

static CborError decode_graph(CborValue *graphMapIt, PlcGraph *outGraph) {
    if (!cbor_value_is_map(graphMapIt)) {
        LOGF("graph is not map, type=%s", cbor_type_name(cbor_value_get_type(graphMapIt)));
        return CborErrorIllegalType;
    }

    // defaults
    outGraph->cycleMs = 10;
    outGraph->nodeCount = 0;

    CborValue val;

    // cycleMs
    if (map_find(graphMapIt, "cycleMs", &val)) {
        LOGF("cycleMs type=%s", cbor_type_name(cbor_value_get_type(&val)));
        uint32_t cyc;
        if (!get_uint32(&val, &cyc)) {
            LOGF("cycleMs parse error");
            return CborErrorIllegalType;
        }
        outGraph->cycleMs = cyc;
    } else {
        LOGF("cycleMs not present");
    }

    // nodes[]

    if (!map_find(graphMapIt, "nodes", &val)) {
        LOGF("nodes missing");
        return CborErrorUnknownType;
    }
    if (!cbor_value_is_array(&val)) {
        LOGF("nodes is not array, type=%s", cbor_type_name(cbor_value_get_type(&val)));
        return CborErrorIllegalType;
    }

    LOGF("enter nodes array");

    // Итерируем массив узлов
    CborValue arrIt;
    CborError err = cbor_value_enter_container(&val, &arrIt);
    if (err != CborNoError) return err;

    uint16_t count = 0;
    while (!cbor_value_at_end(&arrIt)) {
        if (count >= PLC_MAX_NODES) {
            LOGF("CborErrorOutOfMemory");
            return CborErrorOutOfMemory; // слишком много узлов
        }

        if (!cbor_value_is_map(&arrIt)) {
            LOGF("arrIt is not map, type=%s", cbor_type_name(cbor_value_get_type(&arrIt)));
            return CborErrorIllegalType;
        }

        err = decode_single_node(&arrIt, &outGraph->nodes[count]);
        if (err != CborNoError) {
            LOGF("CborError -> decode_single_node");
            return err;
        }

        // перешагиваем к следующему элементу массива
        err = cbor_value_advance(&arrIt);
        if (err != CborNoError) {
            LOGF("CborError -> cbor_value_advance");
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
bool parse_cmd_upload_graph(CborValue root) {

    CborError err;
    // Достаём "graph"
    CborValue valGraph;
    map_find(&root, "graph", &valGraph);
    if (!cbor_value_is_map(&valGraph)) {
        return false;
    }

    PlcGraph tmp;
    err = decode_graph(&valGraph, &tmp);
    if (err != CborNoError) {
        return false;
    }

    plc_upload_graph(&tmp);
    // Если дошли сюда — граф корректен.
    // Кладём во staging и отмечаем флаги.
//    g_stagingGraph = tmp;
//    g_stagingGraphValid = true;
//    g_needSwapGraph = true;

    return true;
}

/* Точка входа из транспорта */
void on_cbor_frame_received(const uint8_t *buf, size_t len, CborReply* out) {
    if (out) out->len = 0;
    CborParser parser;
    CborValue root;
    CborError err;

    err = cbor_parser_init(buf, len, 0, &parser, &root);
    if (err != CborNoError) {
        return;
    }

    // Ждём корневую map
    if (!cbor_value_is_map(&root)) {
        return;
    }

    // Проверяем cmd
    CborValue valCmd;
    map_find(&root, "cmd", &valCmd);
    if (!cbor_value_is_text_string(&valCmd)) {
        return;
    }

    char cmdBuf[32];
    size_t cmdLen = sizeof(cmdBuf) - 1;
    err = cbor_value_copy_text_string(&valCmd, cmdBuf, &cmdLen, NULL);
    if (err != CborNoError) {
        return;
    }
    cmdBuf[cmdLen] = '\0';

    if (strcmp(cmdBuf, "upload_graph") == 0) {
        bool ok = parse_cmd_upload_graph(root);
        send_upload_graph_ack(ok, out);
    }

    if (strcmp(cmdBuf, "activate_graph") == 0) {
        bool ok = plc_request_activate_graph();
        send_activate_graph_ack(ok, out);
    }

    if (strcmp(cmdBuf, "status") == 0) {
        bool ok = plc_request_activate_graph();
        send_status_reply(out);
    }

}








