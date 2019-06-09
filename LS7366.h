#include "MPC5606B.h"

/* IR Register Description */
#define MDR0 		0x08
#define MDR1		0x10
#define DTR			0x18
#define CNTR		0x20
#define OTR			0x28
#define STR			0x30

#define CLR			0x00
#define RD			0x40
#define WR			0x80
#define LD			0xC0

/* MDR0 Register Description */
#define QUADNON		0x00
#define QUADX1		0x01
#define QUADX2		0x02
#define QUADX4		0x03

#define FRMODE		0x00
#define SCMODE		0x04
#define RLMODE		0x08
#define MNMODE		0x0C

#define IDXDIS		0x00
#define IDXLCNT		0x10
#define IDXRCNT		0x20
#define IDXLOTR		0x30

#define ASYNCIDX	0x00
#define SYNCIDX		0x40

#define FCDFACT1	0x00
#define FCDFACT2	0x80

/* MDR1 Register Description */
#define CNT4BT		0x00
#define CNT3BT		0x01
#define CNT2BT		0x02
#define CNT1BT		0x03

#define CNTEN		0x00
#define CNTDIS		0x04

void initLS7366(void);
uint32_t GetEncoderPulse(uint8_t cs);

void initLS7366(void)
{
	writeDataDSPI_0(WR | MDR0, 1, 1);			// Select MDR0, WR_REG
	writeDataDSPI_0(IDXRCNT | QUADX4, 1, 0);	// X4 Count Mode
	Delay(10000);

	writeDataDSPI_0(WR | MDR1, 1, 1);			// Select MDR1, WR_REG
	writeDataDSPI_0(CNTEN | CNT4BT, 1, 0);		// Count Enable, 4 Byte Count Mode
	Delay(10000);

	writeDataDSPI_0(CLR | CNTR, 1, 0);			// Clear CNTR
	Delay(10000);

	writeDataDSPI_0(CLR | STR, 1, 0);			// Select STR, CLR_REG
	Delay(10000);

	writeDataDSPI_0(WR | MDR0, 1, 1);			// Select MDR0, WR_REG
	writeDataDSPI_0(IDXRCNT | QUADX4, 1, 0);	// X4 Count Mode
	Delay(10000);
	
	
	writeDataDSPI_0(WR | MDR0, 2, 1);			// Select MDR0, WR_REG
	writeDataDSPI_0(IDXRCNT | QUADX4, 2, 0);	// X4 Count Mode
	Delay(10000);

	writeDataDSPI_0(WR | MDR1, 2, 1);			// Select MDR1, WR_REG
	writeDataDSPI_0(CNTEN | CNT4BT, 2, 0);		// Count Enable, 4 Byte Count Mode
	Delay(10000);

	writeDataDSPI_0(CLR | CNTR, 2, 0);			// Clear CNTR
	Delay(10000);

	writeDataDSPI_0(CLR | STR, 2, 0);			// Select STR, CLR_REG
	Delay(10000);

	writeDataDSPI_0(WR | MDR0, 2, 1);			// Select MDR0, WR_REG
	writeDataDSPI_0(IDXRCNT | QUADX4, 2, 0);	// X4 Count Mode
	Delay(10000);	
}

uint32_t GetEncoderPulse(uint8_t cs)
{
	uint32_t cnt;
	
	writeDataDSPI_0(0x60, cs, 1);
	cnt = (uint32_t)(writeDataDSPI_0(0xFF, cs, 1) << 24);
	cnt |= (uint32_t)(writeDataDSPI_0(0xFF, cs, 1) << 16);
	cnt |= (uint32_t)(writeDataDSPI_0(0xFF, cs, 1) << 8);
	cnt |= (uint32_t)(writeDataDSPI_0(0xFF, cs, 0));
	
	return cnt;
}
