const {light} = require('zigbee-herdsman-converters/lib/modernExtend');

const definition = {
    zigbeeModel: ['LEDCopperV1'],
    model: 'LEDCopperV1',
    vendor: 'DIY',
    description: 'LED Copper String Light',
    extend: [light({effect: false})],
    icon: 'https://i.imgur.com/t8u7H0D.png',
};

module.exports = definition;
