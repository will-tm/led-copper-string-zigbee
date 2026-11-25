const {light} = require('zigbee-herdsman-converters/lib/modernExtend');

const definition = {
    zigbeeModel: ['LEDCopperSimple'],
    model: 'LEDCopperSimple',
    vendor: 'DIY',
    description: 'LED Copper String Light (single PWM output)',
    extend: [
        light({colorTemp: false, color: false}),
    ],
    ota: true,
    icon: 'https://i.imgur.com/t8u7H0D.png',
};

module.exports = definition;
