const {light, battery} = require('zigbee-herdsman-converters/lib/modernExtend');

const definition = {
    zigbeeModel: ['LEDCopperV1'],
    model: 'LEDCopperV1',
    vendor: 'DIY',
    description: 'LED Copper String Light with Battery',
    extend: [light(), battery()],
    icon: 'https://i.imgur.com/t8u7H0D.png',
};

module.exports = definition;
