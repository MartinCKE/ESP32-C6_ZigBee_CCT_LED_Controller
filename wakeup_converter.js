const fz = require('zigbee-herdsman-converters/converters/fromZigbee');
const tz = require('zigbee-herdsman-converters/converters/toZigbee');
const exposes = require('zigbee-herdsman-converters/lib/exposes');

const ea = exposes.access;

const WAKEUP_CLUSTER = 0xFF10;
const MANUF_CODE = 0x1234;

const ATTR = {
    start_bri: 0x0001,
    end_bri:   0x0002,
    start_ct:  0x0003,
    end_ct:    0x0004,
    fade_ms:   0x0005,
    start:     0x0006,
};

const fzWakeup = {
    cluster: WAKEUP_CLUSTER,
    type: ['attributeReport', 'readResponse'],
    convert: (model, msg) => {
        const out = {};
        const data = msg.data;
        if (!data) return;

        const get = (id) => data[id] ?? data[String(id)] ??
            data[id.toString(16)] ??
            data[id.toString(16).padStart(4, '0')] ??
            data['0x' + id.toString(16).padStart(4, '0')];

        const a1 = get(ATTR.start_bri); if (a1 !== undefined) out.wakeup_start_bri = Number(a1);
        const a2 = get(ATTR.end_bri);   if (a2 !== undefined) out.wakeup_end_bri   = Number(a2);
        const a3 = get(ATTR.start_ct);  if (a3 !== undefined) out.wakeup_start_ct  = Number(a3);
        const a4 = get(ATTR.end_ct);    if (a4 !== undefined) out.wakeup_end_ct    = Number(a4);

        const a5 = get(ATTR.fade_ms);
        if (a5 !== undefined) out.wakeup_fade_time = Math.round(Number(a5) / 60000); // ms -> minutes

        const a6 = get(ATTR.start);
        if (a6 !== undefined) out.wakeup_start = (a6 === true || a6 === 1 || a6 === '1');

        return out;
    },
};

const tzWakeup = {
    key: ['wakeup_start_bri', 'wakeup_end_bri', 'wakeup_start_ct', 'wakeup_end_ct', 'wakeup_fade_time', 'wakeup_start'],
    convertSet: async (entity, key, value) => {
        let attrId;
        let type;
        let payloadValue;

        if (key === 'wakeup_start_bri') { attrId = ATTR.start_bri; type = 0x20; payloadValue = Number(value); } // u8
        else if (key === 'wakeup_end_bri') { attrId = ATTR.end_bri; type = 0x20; payloadValue = Number(value); } // u8
        else if (key === 'wakeup_start_ct') { attrId = ATTR.start_ct; type = 0x21; payloadValue = Number(value); } // u16
        else if (key === 'wakeup_end_ct') { attrId = ATTR.end_ct; type = 0x21; payloadValue = Number(value); } // u16
        else if (key === 'wakeup_fade_time') { attrId = ATTR.fade_ms; type = 0x23; payloadValue = Number(value) * 60000; } // u32 (min->ms)
        else if (key === 'wakeup_start') { attrId = ATTR.start; type = 0x10; payloadValue = value ? 1 : 0; } // bool
        else return;

        // Keep your existing “no manufacturerCode in write” behavior:
        // If you DO need manufCode, add {manufacturerCode: MANUF_CODE} as 3rd arg here.
        await entity.write(WAKEUP_CLUSTER, {[attrId]: {value: payloadValue, type}});
        return {state: {[key]: value}};
    },

    // ✅ This makes zigbee2mqtt/<device>/get work for these keys
    convertGet: async (entity, key) => {
        const lookup = {
            wakeup_start_bri: ATTR.start_bri,
            wakeup_end_bri:   ATTR.end_bri,
            wakeup_start_ct:  ATTR.start_ct,
            wakeup_end_ct:    ATTR.end_ct,
            wakeup_fade_time: ATTR.fade_ms,
            wakeup_start:     ATTR.start,
        };
        const attrId = lookup[key];
        if (attrId === undefined) return;

        // Read usually *does* need manufacturerCode on manuf clusters:
        await entity.read(WAKEUP_CLUSTER, [attrId], {manufacturerCode: MANUF_CODE});
    },
};

const definition = {
    zigbeeModel: ['CCT-SmartLamp'],
    fingerprint: [{manufacturerName: 'CK-Home'}],
    model: 'CCT-SmartLamp-wakeup',
    vendor: 'CK-Home',
    description: 'Custom Wakeup Lamp - Full UI (HA compatible light + sensors)',
    ota: true,

    configure: async (device, coordinatorEndpoint, logger) => {
        const ep = device.getEndpoint(10);
        if (!ep) return;

        // Read initial wakeup attrs so they populate state
        try {
            await ep.read(WAKEUP_CLUSTER,
                [ATTR.start_bri, ATTR.end_bri, ATTR.start_ct, ATTR.end_ct, ATTR.fade_ms, ATTR.start],
                {manufacturerCode: MANUF_CODE},
            );
        } catch (e) {
            if (logger && logger.info) logger.info(`wakeup read failed: ${e}`);
        }
    },

    exposes: [
        // ✅ Most HA-stable light expose (this is the “real” light entity)
        exposes.light()
            .withBrightness()
            .withColorTemp([200, 455]),

        // ✅ Plain numeric sensors so HA always creates entities
        exposes.numeric('temperature', ea.STATE).withUnit('°C'),
        exposes.numeric('humidity', ea.STATE).withUnit('%'),

        // wakeup params
        exposes.numeric('wakeup_start_bri', ea.ALL).withValueMin(0).withValueMax(254).withDescription('Starting brightness'),
        exposes.numeric('wakeup_end_bri', ea.ALL).withValueMin(0).withValueMax(254).withDescription('End brightness'),
        exposes.numeric('wakeup_start_ct', ea.ALL).withValueMin(200).withValueMax(455).withDescription('Starting color temperature'),
        exposes.numeric('wakeup_end_ct', ea.ALL).withValueMin(200).withValueMax(455).withDescription('End color temperature'),
        exposes.numeric('wakeup_fade_time', ea.ALL)
            .withValueMin(0).withValueMax(30).withValueStep(1)
            .withUnit('min')
            .withDescription('Fade duration in minutes'),
        exposes.binary('wakeup_start', ea.ALL, true, false).withDescription('Trigger the wakeup sequence'),
    ],

    fromZigbee: [
        fz.on_off,
        fz.brightness,
        fz.color_colortemp,
        fz.temperature,
        fz.humidity,
        fzWakeup,
    ],

    toZigbee: [
        tz.light_onoff_brightness,
        tz.light_colortemp,
        tzWakeup,
    ],
};

module.exports = [definition];
