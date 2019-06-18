#pragma GCC diagnostic warning "-Wall"

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <time.h>
#include <assert.h>
#include <iostream>
#include <ostream>
#include <string>
#include <set>
#include <vector>
#include <tuple>
#include <system_error>
#include <modbus/modbus.h>
#include <unistd.h>
#include <algorithm> 

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define DIV_ROUND_UP(n, d) (((n) + (d) - 1) / (d))
#define DIV_ROUND_DOWN(n, d) ((n) / (d))
#define ROUND_UP(n, d) (DIV_ROUND_UP(n, d) * (d))
#define ROUND_DOWN(n, d) (DIV_ROUND_DOWN(n, d) * (d))
	
#define print_error(text, ...) printf(text ": Error %d: %s\n", ##__VA_ARGS__, errno, modbus_strerror(errno))
#define ATTR_NON_NULL_PTR	__attribute__ ((nonnull))
	
using namespace std;	

struct device {
	const char *path;

	ATTR_NON_NULL_PTR
	explicit constexpr device(const char *path) : path(path) {}
	virtual ~device() {}

	virtual bool connect() = 0;
	virtual bool connected() const = 0;
};

ATTR_NON_NULL_PTR
static tuple<const char*, int, char, int> parseModbusParams(char *connectionString)
{
	int baud = 9600;
	char parity = 'N';
	int slave = 1;

	if(char *slaveStr = strrchr(connectionString, ':'))
	{
		*slaveStr++ = 0;
		int val = atoi(slaveStr);
		if(val >= 1 && val <= 247)
			slave = val;
		else
			fprintf(stderr, "%s: Slave address has to be between 1 and 247, using default %d\n", connectionString, slave);
	}

	if(char *baudStr = strrchr(connectionString, '@'))
	{
		*baudStr++ = 0;
		char *parityStr;
		long int val = strtol(baudStr, &parityStr, 10);
		if(val > 0 && val <= INT_MAX)
		{
			baud = val;
			switch (*parityStr) {
				case 'N':
				case 'E':
				case 'O': parity = parityStr[0]; break;
				case 0: break;
				default: fprintf(stderr, "%s: unknown characters after baud rate, possible is 'N', 'E' or 'O' for no, even or odd parity. Using default '%c'\n", connectionString, parity);
			}
		} else {
			fprintf(stderr, "%s: baud rate has to be a positive number, using default %d\n", connectionString, baud);
		}
	}
	return {connectionString, baud, parity, slave};
}

struct modbus : public device {
	int slave;
	modbus_t *mb;
	enum {
		SLAVE,
		CREATED		= 1<<0,
		CONNECTED	= 1<<1,
	} state;

	explicit modbus(const char *path, int baud, char parity, int slave) : device(path), slave(slave)
	{
		mb = modbus_new_rtu(path, baud, parity, 8, 1);
		if(!mb)
			throw invalid_argument(modbus_strerror(errno));
		
		state = CREATED;
			
		modbus_set_debug(mb, true);
	}
	constexpr modbus(const modbus *master, int slave) : device(master->path), slave(slave), mb(master->mb), state(SLAVE) {}

	virtual ~modbus()
	{
#warning CONNECTED usw
		if(state & CREATED)
		{
			modbus_free(mb);
		}
	} 	
	
	virtual bool connect()
	{
#warning TODO
		return true;
	}
	virtual bool connected() const
	{
#warning TODO
		return true;
	}
};

struct huawei : public modbus
{
	explicit huawei(const char *path, int baud, char parity, int slave) : modbus(path, baud, parity, slave) {}
	explicit huawei(const modbus *master, int slave) : modbus(master, slave) {}
};



struct datapoint {
	uint_fast64_t value{};
	uint_fast64_t oldValue{};
	bool changed{};
	time_t nextCheck{};

	const uint_fast16_t address;	
	const enum Type
	{
		uint8   = 1,
		boolean = 1,
		uint16  = 2,
		uint32  = 4,
		uint64  = 8,
		timeBCD = uint64 + 32,
		int8    = uint8  + 16,
		int16   = uint16 + 16,
		int32   = uint32 + 16,
		int64   = uint64 + 16,
	} type;
	const uint_fast16_t interval;

	Type parseType(const string& type);
	constexpr uint_fast8_t getLength(void) const
	{
		return (uint_fast8_t)type & 15;
	}
	constexpr bool getSigned(void) const
	{
		return ((uint_fast8_t)type & 16) == 16;
	}
		
//	void fromViessmann(uint8_t bytes[sizeof(T)]);
//	void fromModbus(uint16_t words[DIV_ROUND_UP(sizeof(T), 2)]);

	datapoint(uint_fast16_t address, const string& type, uint_fast16_t interval) : address(address), type(parseType(type)), interval(interval) {} 
	constexpr datapoint(uint_fast16_t address, Type type, uint_fast16_t interval) : address(address), type(type), interval(interval) {} 	
	
	bool operator<(const datapoint &other) const
	{	// This prohibits overlapping entries in set<datapoint>
		return (address + getLength() <= other.address);
	}

	friend std::ostream& operator <<(std::ostream& os, datapoint const& m)
	{
		if(m.getSigned())
			return os << (int_fast64_t)m.value;
		else
			return os << m.value;
	}
};

datapoint::Type datapoint::parseType(const string& type)
{
	if(type == "uint8" || type == "unsigned char" || type == "byte")
		return uint8;
	if(type == "int8" || type == "char" || type == "signed char")
		return int8;
	if(type == "uint16" || type == "unsigned word" || type == "unsigned short")
		return uint16;
	if(type == "int16" || type == "word" || type == "short")
		return int16;
	if(type == "uint32" || type == "unsigned dword" || type == "unsigned long")
		return uint32;
	if(type == "int32" || type == "dword" || type == "long")
		return int32;
	if(type == "uint64")
		return uint64;
	if(type == "int64")
		return int64;
	if(type == "timeBCD")
		return timeBCD;
	throw std::errc::invalid_argument;
}



int printUsage(void)
{
	fprintf(stderr, "at least one parameter is needed, usuage is \n");
	fprintf(stderr, "	-g path : GPIO on path\n");		
	fprintf(stderr, "	-h path[@9600[N81]][:1] : Huawei SUN2000 Inverter on path, rs485 speed and slave address\n");
	fprintf(stderr, "	-s path : Energy Meter speaking SML protocol on path\n");
	fprintf(stderr, "	-v path : Viessmann central heating on path\n");
	return 1;
}

	
int main(int argc, char **argv)
{
	vector<device*> devices;
	
	for(int opt; (opt = getopt(argc, argv, "g:h:s:v:")) != -1;)
	{

		switch(opt)
		{
			case 'g': {

			} break;
			case 'h': {
				auto [path, baud, parity, slave] = parseModbusParams(optarg);
				auto samePath = find_if(devices.begin(), devices.end(), [path] (const device *i) { return i->path == path; } );
				if(samePath == devices.end())
				{
					try {
						devices.push_back(new huawei(path, baud, parity, slave));
					}
					catch (const invalid_argument &e)
					{
						fprintf(stderr, "cannot add device %s: %s", path, e.what());
					}
				} else {
					modbus *master = dynamic_cast<modbus *>(*samePath);
					if(master && master->slave != slave)
						devices.push_back(new huawei(master, slave));
					else
						fprintf(stderr, "cannot add device %s: connection blocked by other device\n", path);
				}				
			} break;
			case 's': break;
			case 'v': break;
			default: return printUsage();
		}
	}

	
	modbus_t *mb = modbus_new_rtu("/dev/ttyS0", 19200, 'N', 8, 1);
	if(!mb)
		return print_error("modbus_new_rtu");

	modbus_set_debug(mb, TRUE);

	modbus_rtu_set_serial_mode(mb, MODBUS_RTU_RS485); // ignore return value

	if(modbus_set_slave(mb, 1))
		return print_error("modbus_set_slave");

	if(modbus_connect(mb))
		/*return*/ print_error("modbus_connect");
	
	
	set<datapoint> datapoints;
	datapoints.emplace(3, "uint16", 600);
	if(!datapoints.emplace(4, "uint16", 600).second)
		printf("Wert ist schon in Array\n");
	datapoints.emplace(0, "uint16", 600);
	datapoints.emplace(10, "uint16", 600);
	




	// Vorbereiten für versand, Liste mit Ketten
	struct dataset {
		const uint_fast16_t address;
		uint_fast16_t length;
		vector<pair<const uint_fast16_t, const datapoint&>> offsets;
		dataset(uint_fast16_t address, uint_fast16_t length, const datapoint& datapoint) : address(address), length(length), offsets{{0, datapoint}} {}
	};
	vector<dataset> datasets;

	time_t now = time(NULL);
	for(const datapoint& entry : datapoints)
	{
		if(entry.nextCheck <= now && (datasets.empty() || (datasets.back().address + datasets.back().length + 10) < entry.address))
		{
			datasets.emplace_back(entry.address, entry.getLength(), entry);
		} else {
			uint_fast16_t offset = entry.address - datasets.back().address;
			datasets.back().length = offset + entry.getLength();
			datasets.back().offsets.emplace_back(offset, entry);
		}
	}

	for(auto entry : datasets)
	{
		printf("start: %lu, length: %lu\n", entry.address, entry.length);
		for(auto entry2 : entry.offsets)
		{
			printf("    offset: %lu length: %d value: ", entry2.first, entry2.second.getLength());
			cout << entry2.second << endl;
		}
		
	}
	return 0;
}
