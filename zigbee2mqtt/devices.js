const devices = [
    // Custom sensor device
    {
        // simplest configuration
        // the end device auto binds to the coordinator.
        // temperature and humidity updates controlled by zstack
        // battery requires a manual report to be sent ot coordinator.
        zigbeeModel: ['SA-AHT10'],
        model: 'AHT10 Sensor',
        vendor: 'Sprinfield',
        description: 'AHT10 Temperatuer and Humidity Sensor',
        supports: '',
        fromZigbee: [fz.temperature, fz.humidity, fz.battery],
        toZigbee: [],
        exposes: [e.temperature(), e.humidity(), e.battery()],
    }
]
