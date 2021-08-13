/*#include "feynman_usb.h"
#include "feynman_usb_proto.h"

#include "feynman_cmd_module.h"
#include "feynman_upgrade_module.h"
*/
#include "feynman_sdk.h"
#ifdef WIN32
#include <io.h>
#include <windows.h>
#include <conio.h>
#include <tchar.h>
#endif

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <time.h>

#define UPGRADE_MAX_BLK_SIZE (1024 * 2)
#pragma warning(disable : 4996)
#define log_printf printf
int update_result = 0;
int8_t upgrade_update_flag[256] = {0};
/*
return : -1:�ļ���ʧ�ܻ��ļ�������
         -2:ϵͳ�ڴ����
         -3:��������ֹͣ����
         0:�ɹ�
*/

int feynman_upgrade(int upgrade_type, char *dst_path, char *dst_filename, char *local_filename)
{
    int ret_code = -1;
    FILE *fp = fopen(local_filename, "rb");
    log_printf("feynman_upgrade:upgrade_type[%d], dst_path[%s], dst_filename[%s], local_filename[%s]\r\n",
               upgrade_type, dst_path, dst_filename, local_filename);

    if (fp)
    {
        size_t file_size = 0, read_offset = 0, read_len = 0;
        int total_blk = 0, current_blk = 1;
        char *file_data = NULL;
        int updated = 0;

        fseek(fp, 0, SEEK_END);
        file_size = ftell(fp);
        fseek(fp, 0, SEEK_SET);

        total_blk = (file_size + UPGRADE_MAX_BLK_SIZE - 1) / UPGRADE_MAX_BLK_SIZE;

        file_data = (char *)malloc(sizeof(s_feynman_upgrade_data) + UPGRADE_MAX_BLK_SIZE);

        if (file_data)
        {
            s_feynman_upgrade_data *header = (s_feynman_upgrade_data *)file_data;
            ret_code = 0;

            while (read_offset < file_size)
            {
                memset(file_data, 0, sizeof(s_feynman_upgrade_data) + UPGRADE_MAX_BLK_SIZE);

                read_len = (file_size - read_offset) > UPGRADE_MAX_BLK_SIZE ? UPGRADE_MAX_BLK_SIZE : file_size - read_offset;
                fread(file_data + sizeof(s_feynman_upgrade_data), 1, read_len, fp);

                header->packet_numbers = total_blk;
                header->curr_packet_numbers = current_blk++;
                memset(header->path, 0, sizeof(header->path));
                memset(header->name, 0, sizeof(header->name));
                strcpy(header->path, dst_path);
                strcpy(header->name, dst_filename);
                header->data_len = read_len;
                log_printf("upgrade : total_blk[%d], current_blk[%d], offset[%d], data_len[%d]\r",
                           header->packet_numbers, header->curr_packet_numbers, read_offset, header->data_len);
                read_offset += read_len;

                int32_t usb_send_upgrade_and_get_ack(FEYNMAN_UPGRADE_SUB_TYPE type, void *data, int32_t size, int32_t timeout);
                if ((updated = usb_send_upgrade_and_get_ack(
                         (upgrade_type == 0) ? FEYNMAN_COMMAND_USB_UPGRADE_FEYNMAN : ((upgrade_type == 1) ? FEYNMAN_COMMAND_USB_UPGRADE_LIB : FEYNMAN_COMMAND_USB_UPGRADE_FILE),
                         header, sizeof(s_feynman_upgrade_data) + read_len, 0)) != 1)
                {
                    if (update_result == -1)
                    {
                        //need retransfer
                        fseek(fp, 0 - read_len, SEEK_CUR);
                        read_offset -= read_len;
                        current_blk--;
                    }
                    else if (update_result == -2)
                    {
                        //occr error ,stop
                        ret_code = -3;
                        break;
                    }
                    else
                    {
                        //success, do nothing
                    }
                }
            }
        }
        else
        {
            ret_code = -2;
        }
        fclose(fp);
    }

    return ret_code;
}

int32_t get_upgrade_update_flag_index(FEYNMAN_UPGRADE_SUB_TYPE type)
{
    int32_t index = -1;

    switch (type)
    {
    case FEYNMAN_COMMAND_USB_UPGRADE_FEYNMAN:
    case FEYNMAN_COMMAND_USB_UPGRADE_FEYNMAN_RETURN:
    {
        index = 0;
        break;
    }
    case FEYNMAN_COMMAND_USB_UPGRADE_LIB:
    case FEYNMAN_COMMAND_USB_UPGRADE_LIB_RETURN:
    {
        index = 1;
        break;
    }
    case FEYNMAN_COMMAND_USB_UPGRADE_FILE:
    case FEYNMAN_COMMAND_USB_UPGRADE_FILE_RETURN:
    {
        index = 2;
        break;
    }
    }

    return index;
}

void usb_upgrade_set_update_mask(FEYNMAN_UPGRADE_SUB_TYPE type)
{
    int32_t index = get_upgrade_update_flag_index(type);
    if (index >= 0)
    {
        upgrade_update_flag[index] = 0xA5;
    }
}
/*
* return 1 - updated, 0 - not updated, -1 - updated but error
*/
int32_t usb_upgrade_mask_is_updated(FEYNMAN_UPGRADE_SUB_TYPE type)
{
    int32_t index = get_upgrade_update_flag_index(type);
    if (index >= 0)
    {
        return (upgrade_update_flag[index] == 0x00) ? 1 : ((upgrade_update_flag[index] == 0xA5) ? 0 : (-1));
    }

    return 0;
}
static void feynman_usb_send_upgrade(uint16_t type, void *p_data, int data_length)
{
    char *tmp = (char *)malloc(sizeof(FEYNMAN_USBHeaderDataPacket) + data_length);
    FEYNMAN_USBHeaderDataPacket *p_cmd = (FEYNMAN_USBHeaderDataPacket *)tmp;
    sprintf((char *)p_cmd->magic, (char *)"NEXT_VPU");
    p_cmd->type = FEYNMAN_UPGRADE_DATA;
    p_cmd->sub_type = type;
    p_cmd->checksum = 0xA5A5;
    p_cmd->len = data_length;

    if (data_length)
    {
        memcpy(p_cmd->data, p_data, data_length);
    }
    extern int usb_hal_write(uint8_t * data, int len, int *bytesTransffered, int timeout);
    int transfered = 0;
    usb_hal_write((uint8_t *)p_cmd, sizeof(FEYNMAN_USBHeaderDataPacket) + data_length, &transfered, 2000);
    /*	if (usb_para.m_dev_handle != NULL)
	{
		usb_bulk_write(usb_para.m_dev_handle, usb_para.usb_ep_out, (char*)p_cmd, sizeof(FEYNMAN_USBHeaderDataPacket) + data_length, 0);
	}*/

    free(tmp);
}

int32_t usb_send_upgrade_and_get_ack(FEYNMAN_UPGRADE_SUB_TYPE type, void *data, int32_t size, int32_t timeout)
{
    int32_t updated = 0;

    usb_upgrade_set_update_mask(type);

    feynman_usb_send_upgrade(type, data, size);
    while (0 == (updated = usb_upgrade_mask_is_updated(type)))
    {
    }
    return updated;
}
