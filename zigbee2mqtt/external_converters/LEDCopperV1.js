const {light, battery, deviceEndpoints, numeric} = require('zigbee-herdsman-converters/lib/modernExtend');

const definition = {
    zigbeeModel: ['LEDCopperV1'],
    model: 'LEDCopperV1',
    vendor: 'DIY',
    description: 'LED Copper String Light with Battery',
    extend: [
        deviceEndpoints({endpoints: {light: 1}}),
        light({colorTemp: false, color: false}),
        battery(),
        numeric({
            name: 'voltage_mv',
            cluster: 'genPowerCfg',
            attribute: {ID: 0xFF00, type: 0x21}, // uint16
            description: 'Battery voltage in millivolts',
            access: 'STATE_GET',
            unit: 'mV',
            reporting: {min: 3600, max: 7200, change: 100},
        }),
    ],
    icon: 'https://i.imgur.com/t8u7H0D.png',
};

module.exports = definition;
