#define _GNU_SOURCE 1
#include <stdio.h>
#include <stdint.h>
#include <errno.h>
#include <modbus/modbus.h>
#include <assert.h>
#include <sys/timerfd.h>
#include <time.h>
#include <unistd.h>
#include <stdlib.h>
#include <inttypes.h>

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

#define SLAVE_ADDRESS 1

#define print_error(text, ...) printf(text ": Error %d: %s\n", ##__VA_ARGS__, errno, modbus_strerror(errno))

static const struct {
	const int address;
	const int length;
} request1s[] = {
	{ 32262, 12 },	// PV1 - PV6 Voltage + Current
	{ 32314, 4 },	// PV7 - PV8 Voltage + Current
	{ 32287, 1 },	// Inverter Status
	{ 32290, 4 },	// Active and Reactive Power
	{ 50001, 9 },	// Alarms
};
#define request1s_length (12+4+1+4+9)

int main(void)
{
	modbus_t *mb = modbus_new_rtu("/dev/ttyUSB0", 19200, 'N', 8, 1);
	if(!mb)
		return print_error("modbus_new_rtu");

//	modbus_set_debug(mb, TRUE);

	if(modbus_rtu_set_serial_mode(mb, MODBUS_RTU_RS485))
		print_error("modbus_set_serial_mode");

	if(modbus_set_slave(mb, SLAVE_ADDRESS))
		return print_error("modbus_set_slave");

	if(modbus_connect(mb))
		return print_error("modbus_connect");

	// Setup timer
	int timer = timerfd_create(CLOCK_MONOTONIC, 0);
	if(timer < 0)
		return print_error("timerfd_create");
		
	const struct itimerspec interval = {.it_value = {1}, .it_interval = {1}};	// start in 1 sek, interval 1s
	if(timerfd_settime(timer, 0, &interval, NULL))
		return print_error("timerfd_settime");

	uint64_t timer_expired;
	while(read(timer, &timer_expired, sizeof(timer_expired)) == sizeof(timer_expired))
	{
		if(timer_expired != 1)
			printf("timer interval skipped %" PRIu64 " times\n", timer_expired);
			
		uint16_t result1s[request1s_length], *result = result1s;
		for(size_t i = 0; i < ARRAY_SIZE(request1s); i++)
		{
			if(modbus_read_registers(mb, request1s[i].address, request1s[i].length, result) < request1s[i].length)
			{
				print_error("modbus_read_registers(%d, %d)", request1s[i].address, request1s[i].length);
				goto nextRead;
			}

			for(size_t j = 0; j < request1s[i].length; j++)
			{
				printf("%d: %d\n", request1s[i].address + j, result[j]);
			}

			result += request1s[i].length;
		}
			

	nextRead:;
	}

	modbus_close(mb);
	modbus_free(mb);
	return 0;
}
