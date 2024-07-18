#include <stdint.h>
#include <drivers.h>

// #define UID_BASE ((uint32_t*)0x1FF0F420)


struct UID {
    uint16_t    x;          // waffer x-coordinate
    uint16_t    y;          // waffer y-coordinate
    uint8_t     w_num;      // waffer number
    char        lot_num[7]; // Lot number
};


union ChipUIDMap
{
    uint32_t int32[3];
    UID      id;
};


void print_uid()
{   
    //load uid
    ChipUIDMap uid;

    uid.int32[0] = *(uint32_t *)(UID_BASE + 0);
    uid.int32[1] = *(uint32_t *)(UID_BASE + 4);
    uid.int32[2] = *(uint32_t *)(UID_BASE + 8);

    terminal << uid.int32[0] << "\n";
    terminal << uid.int32[1] << "\n";
    terminal << uid.int32[2] << "\n";

    terminal << "waffer position " << uid.id.x << " " << uid.id.y << "\n";
    terminal << "waffer number   " << uid.id.w_num << "\n";
    terminal << "lot number      "; 
    for (unsigned int i = 0; i < 7; i++)
    {
        terminal << (int)uid.id.lot_num[i] << " ";
    }
    terminal << "\n\n\n";
}