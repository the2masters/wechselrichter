#include <stdio.h>
#include <stdint.h>
#include <errno.h>
#include <modbus.h>
#include <assert.h>

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

#define SLAVE_ADDRESS 1

int print_error(const char *text)
{
	printf("Fehler %d bei %s: %s\n", errno, text, modbus_strerror(errno));
	return errno;
}

static const struct {
	const int address;
	const int length;
} request1s[] = {
	{ 32262, 12 },	// PV1 - PV6 Voltage + Current
	{ 32314, 4 },	// PV7 - PV8 Voltage + Current
	{ 32287, 1 },	// Inverter Status
	{ 32290, 2 },	// Active and Reactive Power
	{ 50001, 9 },	// Alarms
};
#define lengthMAX 12

int main(void)
{
	modbus_t *mb = modbus_new_rtu("/dev/ttyS0", 19200, 'N', 8, 1);
	if(!mb)
		return print_error("modbus_new_rtu");

//	modbus_set_debug(mb, TRUE);

	if(modbus_rtu_set_serial_mode(mb, MODBUS_RTU_RS485))
		print_error("modbus_set_serial_mode");

	if(modbus_set_slave(mb, SLAVE_ADDRESS))
		return print_error("modbus_set_slave");

	if(modbus_connect(mb))
		return print_error("modbus_connect");

	uint16_t result1s[lengthMAX];
	for(size_t i = 0; i < ARRAY_SIZE(request1s); i++)
	{
		assert(request1s[i].length <= lengthMAX);
//		if(modbus_read_registers(mb, request1s[i].address, request1s[i].length, result1s) < request1s[i].length)
//			return print_error("modbus_read_registers");
		printf("%d[%d]\n", request1s[i].address, request1s[i].length);

		for(int j = 0; j < request1s[i].length; j++)
			printf("%d: %d\n", request1s[i].address + j, result1s[j]);
	}

	modbus_close(mb);
	modbus_free(mb);
	return 0;
}
