#ifndef __MEMORY_H__
#define __MEMORY_H__

#include <stdint.h>

//--------------------------------------------------------------------
// Abstract interface for memories
//--------------------------------------------------------------------
class Memory
{
public:  
    virtual void        reset(void) = 0;
    virtual uint32_t    load(uint32_t address, int width, bool signedLoad) = 0;
    virtual void        store(uint32_t address, uint32_t data, int width) = 0;
};

//-----------------------------------------------------------------
// Simple little endian memory
//-----------------------------------------------------------------
class SimpleMemory: public Memory
{
public:
    SimpleMemory(int size)
    {
        Mem = new uint32_t[(size + 3)/4];
        Size = size;
    }
    SimpleMemory(uint8_t * buf, int size)
    {
        Mem = (uint32_t*)buf;
        Size = size;
    }    

    virtual void reset(void)
    {
        memset(Mem, 0, Size);
    }

    virtual uint32_t load(uint32_t address, int width, bool signedLoad)
    {
        uint32_t data = 0;

        switch (width)
        {
            case 4:
                assert(!(address & 3));
                data = Mem[address / 4];
            break;
            case 2:
                assert(!(address & 1));

                if (address & 2)
                    data = (Mem[address / 4] >> 16)  & 0xFFFF;
                else
                    data = (Mem[address / 4] >> 0) & 0xFFFF;

                if (signedLoad)
                    if (data & (1 << 15))
                        data |= 0xFFFF0000;
            break;
            case 1:
                switch (address & 3)
                {
                    case 3:
                        data = (Mem[address / 4] >> 24) & 0xFF;
                    break;
                    case 2:
                        data = (Mem[address / 4] >> 16) & 0xFF;
                    break;
                    case 1:
                        data = (Mem[address / 4] >> 8) & 0xFF;
                    break;
                    case 0:
                        data = (Mem[address / 4] >> 0) & 0xFF;
                    break;
                }

                if (signedLoad)
                    if (data & (1 << 7))
                        data |= 0xFFFFFF00;
            break;
        }

        return data;
    }

    virtual void store(uint32_t address, uint32_t data, int width)
    {
        switch (width)
        {
            case 4:
                assert(!(address & 3));
                Mem[address / 4] = data;
            break;
            case 2:
                assert(!(address & 1));
                if (address & 2)
                    Mem[address / 4] = (Mem[address / 4] & 0x0000FFFF) | ((data << 16) & 0xFFFF0000);
                else
                    Mem[address / 4] = (Mem[address / 4] & 0xFFFF0000) | ((data << 0)  & 0x0000FFFF);                
            break;
            case 1:
                switch (address & 3)
                {
                    case 3:
                        Mem[address / 4] = (Mem[address / 4] & 0x00FFFFFF) | ((data << 24) & 0xFF000000);
                    break;
                    case 2:
                        Mem[address / 4] = (Mem[address / 4] & 0xFF00FFFF) | ((data << 16) & 0x00FF0000);
                    break;
                    case 1:
                        Mem[address / 4] = (Mem[address / 4] & 0xFFFF00FF) | ((data << 8) & 0x0000FF00);                    
                    break;
                    case 0:
                        Mem[address / 4] = (Mem[address / 4] & 0xFFFFFF00) | ((data << 0) & 0x000000FF);
                    break;
                }
            break;
        }
    }

private:
    uint32_t *Mem;
    int      Size;
};

#endif
