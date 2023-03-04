#include "tas2780.h"

void tas2780_init(struct tas2780_context * p_context, uint16_t device_address)
{
    p_context->device_address = device_address;
    p_context->book_index = 0x0;
    p_context->page_index = 0x0;
}

void tas2780_set_page(struct tas2780_context * p_context, uint8_t page_index)
{
    p_context->page_index = page_index;
}

void tas2780_set_book(struct tas2780_context * p_context, uint8_t book_index)
{
    p_context->book_index = book_index;
}