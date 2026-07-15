
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "include/ps3.h"
#include "include/ps3_int.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "stack/gap_api.h"
#include "stack/bt_types.h"
#include "stack/l2c_api.h"
#include "osi/allocator.h"

#define PS3_TAG           "PS3_L2CAP"
#define L2CAP_TAG         "L2CAP_TAG"
#define L2CAP_DATA_LEN     100

#define PS3_L2CAP_ID_HIDC 0x40
#define PS3_L2CAP_ID_HIDI 0x41


/********************************************************************************/
/*              L O C A L    F U N C T I O N     P R O T O T Y P E S            */
/********************************************************************************/
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)

static void ps3_l2cap_init_service( char *name, uint16_t psm, uint8_t security_id);
static void ps3_l2cap_deinit_service( char *name, uint16_t psm );
static void ps3_l2cap_connect_ind_cback (BD_ADDR  bd_addr, uint16_t l2cap_cid, uint16_t psm, uint8_t l2cap_id);
static void ps3_l2cap_connect_cfm_cback (uint16_t l2cap_cid, uint16_t result);
static void ps3_l2cap_config_ind_cback (uint16_t l2cap_cid, tL2CAP_CFG_INFO *p_cfg);
static void ps3_l2cap_config_cfm_cback (uint16_t l2cap_cid, tL2CAP_CFG_INFO *p_cfg);
static void ps3_l2cap_disconnect_ind_cback (uint16_t l2cap_cid, bool ack_needed);
static void ps3_l2cap_disconnect_cfm_cback (uint16_t l2cap_cid, uint16_t result);
static void ps3_l2cap_data_ind_cback (uint16_t l2cap_cid, BT_HDR *p_msg);
static void ps3_l2cap_congest_cback (uint16_t cid, bool congested);

#endif

/********************************************************************************/
/*                         L O C A L    V A R I A B L E S                       */
/********************************************************************************/
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0) 

static const tL2CAP_APPL_INFO dyn_info = {
    ps3_l2cap_connect_ind_cback,
    ps3_l2cap_connect_cfm_cback,
    NULL,
    ps3_l2cap_config_ind_cback,
    ps3_l2cap_config_cfm_cback,
    ps3_l2cap_disconnect_ind_cback,
    ps3_l2cap_disconnect_cfm_cback,
    NULL,
    ps3_l2cap_data_ind_cback,
    ps3_l2cap_congest_cback,
    NULL
} ;

static tL2CAP_CFG_INFO ps3_cfg_info;
bool is_connected = false;

#else

struct _L2CAP_channel_info{
    char *name,                 
    uint16_t psm,             
    uint8_t security_id,
    uint32_t connection_handle, 
    int8_t l2cap_state,
    int32_t fd,
    TaskHandle_t rd_task_handle, 
};
typedef struct _L2CAP_channel_info L2CAP_channel_info;

#define PS3_L2CAP_channel 2

L2CAP_channel_info  channel_Info[PS3_L2CAP_channel] = {0};

#endif



/********************************************************************************/
/*                      P U B L I C    F U N C T I O N S                        */
/********************************************************************************/

/*******************************************************************************
**
** Function         ps3_l2cap_init_services
**
** Description      This function initialises the required L2CAP services.
**
** Returns          void
**
*******************************************************************************/
void ps3_l2cap_init_services()
{
    ps3_l2cap_init_service( "PS3-HIDC", BT_PSM_HIDC, BTM_SEC_SERVICE_FIRST_EMPTY   );
    ps3_l2cap_init_service( "PS3-HIDI", BT_PSM_HIDI, BTM_SEC_SERVICE_FIRST_EMPTY+1 );
}

/*******************************************************************************
**
** Function         ps3_l2cap_deinit_services
**
** Description      This function deinitialises the required L2CAP services.
**
** Returns          void
**
*******************************************************************************/
void ps3_l2cap_deinit_services()
{
    ps3_l2cap_deinit_service( "PS3-HIDC", BT_PSM_HIDC );
    ps3_l2cap_deinit_service( "PS3-HIDI", BT_PSM_HIDI );
}


/*******************************************************************************
**
** Function         ps3_l2cap_send_hid
**
** Description      This function sends the HID command using the L2CAP service.
**
** Returns          void
**
*******************************************************************************/
void ps3_l2cap_send_hid( hid_cmd_t *hid_cmd, uint8_t len )
{
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
    uint8_t result;
    BT_HDR  *p_buf;

    p_buf = (BT_HDR *)osi_malloc(BT_DEFAULT_BUFFER_SIZE);

    if( !p_buf ){
        ESP_LOGE(PS3_TAG, "[%s] allocating buffer for sending the command failed", __func__);
    }

    p_buf->len = len + ( sizeof(*hid_cmd) - sizeof(hid_cmd->data) );
    p_buf->offset = L2CAP_MIN_OFFSET;

    memcpy ((uint8_t *)(p_buf + 1) + p_buf->offset, (uint8_t*)hid_cmd, p_buf->len);

    result = L2CA_DataWrite( PS3_L2CAP_ID_HIDC, p_buf );

    if (result == L2CAP_DW_SUCCESS)
        ESP_LOGI(PS3_TAG, "[%s] sending command: success", __func__);

    if (result == L2CAP_DW_CONGESTED)
        ESP_LOGW(PS3_TAG, "[%s] sending command: congested", __func__);

    if (result == L2CAP_DW_FAILED)
        ESP_LOGE(PS3_TAG, "[%s] sending command: failed", __func__);

#else

    int size = 0;
    int fd = 0;
    uint8_t *l2cap_wr_data = NULL;
    uint16_t send_len = len + ( sizeof(*hid_cmd) - sizeof(hid_cmd->data) );

    for(int i = 0; i < PS3_L2CAP_channel; i ++){
        if(channel_Info[i].psm == BT_PSM_HIDC &&
           channel_Info[i].l2cap_state == ESP_BT_L2CAP_OPEN_EVT){
            fd = channel_Info[i].fd;
            break;
        }
    }

    if (fd == 0) {
        ESP_LOGW(L2CAP_TAG, "HIDC channel not open, cannot send");
        return;
    }

    l2cap_wr_data = malloc(BT_DEFAULT_BUFFER_SIZE);
    if (!l2cap_wr_data) {
        ESP_LOGE(L2CAP_TAG, "malloc l2cap_wr_data failed, fd:%d", fd);
        goto done;
    }

    memset(l2cap_wr_data, 0, BT_DEFAULT_BUFFER_SIZE);
    memcpy(l2cap_wr_data, (uint8_t*)hid_cmd, send_len);

    /*
        * The write function is blocked until all the target length of data has been sent to the lower layer
        * successfully an error occurs.
        */
    size = write(fd, l2cap_wr_data, send_len);
    if (size == -1) {
        //do nothing...
    } else if (size == 0) {
        /*write fail due to ringbuf is full, retry after 500 ms*/
        vTaskDelay(pdMS_TO_TICKS(500));
    } else {
        ESP_LOGI(L2CAP_TAG, "fd = %d  data_len = %d", fd, size);
        vTaskDelay(pdMS_TO_TICKS(50));
    }

done:
    if (l2cap_wr_data) {
        free(l2cap_wr_data);
    }

#endif

}


/********************************************************************************/
/*                      L O C A L    F U N C T I O N S                          */
/********************************************************************************/

/*******************************************************************************
**
** Function         ps3_l2cap_init_service
**
** Description      This registers the specified bluetooth service in order
**                  to listen for incoming connections.
**
** Returns          void
**
*******************************************************************************/
static void ps3_l2cap_init_service( char *name, uint16_t psm, uint8_t security_id)
{

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0) 
    /* Register the PSM for incoming connections */
    if (!L2CA_Register(psm, (tL2CAP_APPL_INFO *) &dyn_info)) {
        ESP_LOGE(PS3_TAG, "%s Registering service %s failed", __func__, name);
        return;
    }

    /* Register with the Security Manager for our specific security level (none) */
    if (!BTM_SetSecurityLevel (false, name, security_id, 0, psm, 0, 0)) {
        ESP_LOGE (PS3_TAG, "%s Registering security service %s failed", __func__, name);\
        return;
    }

#else
    /*转存L2CAP信道注册信息(Save L2CAP channel registration information)*/
    for(int i = 0; i < PS3_L2CAP_channel; i ++){
        /*找到首个未写入信道配置的数组元素，并写入
        (Find the first array element not yet written to the channel configuration and write it)*/
        if(channel_Info[i].psm == 0){
            channel_Info[i].name = name;
            channel_Info[i].psm = psm;
            channel_Info[i].security_id = ESP_BT_L2CAP_SEC_NONE; 
            channel_Info[i].l2cap_state = -1;
            channel_Info[i].fd = 0;
            channel_Info[i].rd_task_handle = NULL;
            break;
        }
    }

    /*使用此变量避免多次初始化(Use this Variable to avoid multiple initializations)*/
    static uint8_t Registration_num = 0; 
    Registration_num ++;

    if(Registration_num == PS3_L2CAP_channel){

        /*注册L2CAP事件回调(Register L2CAP event callback)*/
        if ((ret = esp_bt_l2cap_register_callback(esp_bt_l2cap_cb)) != ESP_OK) {
            ESP_LOGE(L2CAP_TAG, "%s l2cap register failed: %s", __func__, esp_err_to_name(ret));
            return;
        }

        /*初始化L2CAP(Initialize L2CAP)*/
        if ((ret = esp_bt_l2cap_init()) != ESP_OK) {
            ESP_LOGE(L2CAP_TAG, "%s l2cap init failed: %s", __func__, esp_err_to_name(ret));
            return;
        }
    }

#endif  

    ESP_LOGI(PS3_TAG, "[%s] Service %s Initialized", __func__, name);
}


/*******************************************************************************
**
** Function         ps3_l2cap_deinit_service
**
** Description      This deregisters the specified bluetooth service.
**
** Returns          void
**
*******************************************************************************/
static void ps3_l2cap_deinit_service( char *name, uint16_t psm )
{
    /* Deregister the PSM from incoming connections */
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
    L2CA_Deregister(psm);

#else
    esp_bt_l2cap_stop_srv(psm);

#endif

    ESP_LOGI(PS3_TAG, "[%s] Service %s Deinitialized", __func__, name);
}

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0) 

/*******************************************************************************
**
** Function         ps3_l2cap_connect_ind_cback
**
** Description      This the L2CAP inbound connection indication callback function.
**
** Returns          void
**
*******************************************************************************/
static void ps3_l2cap_connect_ind_cback (BD_ADDR  bd_addr, uint16_t l2cap_cid, uint16_t psm, uint8_t l2cap_id)
{
    ESP_LOGI(PS3_TAG, "[%s] bd_addr: %s\n  l2cap_cid: 0x%02x\n  psm: %d\n  id: %d", __func__, bd_addr, l2cap_cid, psm, l2cap_id );

    /* Send connection pending response to the L2CAP layer. */
    L2CA_CONNECT_RSP (bd_addr, l2cap_id, l2cap_cid, L2CAP_CONN_PENDING, L2CAP_CONN_PENDING, NULL, NULL);

    /* Send response to the L2CAP layer. */
    L2CA_CONNECT_RSP (bd_addr, l2cap_id, l2cap_cid, L2CAP_CONN_OK, L2CAP_CONN_OK, NULL, NULL);

    /* Send a Configuration Request. */
    L2CA_CONFIG_REQ (l2cap_cid, &ps3_cfg_info);
}


/*******************************************************************************
**
** Function         ps3_l2cap_connect_cfm_cback
**
** Description      This is the L2CAP connect confirmation callback function.
**
**
** Returns          void
**
*******************************************************************************/
static void ps3_l2cap_connect_cfm_cback(uint16_t l2cap_cid, uint16_t result)
{
    ESP_LOGI(PS3_TAG, "[%s] l2cap_cid: 0x%02x\n  result: %d", __func__, l2cap_cid, result );
}


/*******************************************************************************
**
** Function         ps3_l2cap_config_cfm_cback
**
** Description      This is the L2CAP config confirmation callback function.
**
**
** Returns          void
**
*******************************************************************************/
void ps3_l2cap_config_cfm_cback(uint16_t l2cap_cid, tL2CAP_CFG_INFO *p_cfg)
{
    ESP_LOGI(PS3_TAG, "[%s] l2cap_cid: 0x%02x\n  p_cfg->result: %d", __func__, l2cap_cid, p_cfg->result );

    /* The PS3 controller is connected after    */
    /* receiving the second config confirmation */
    is_connected = l2cap_cid == PS3_L2CAP_ID_HIDI;

    if(is_connected){
        ps3Enable();
    }
}


/*******************************************************************************
**
** Function         ps3_l2cap_config_ind_cback
**
** Description      This is the L2CAP config indication callback function.
**
**
** Returns          void
**
*******************************************************************************/
void ps3_l2cap_config_ind_cback(uint16_t l2cap_cid, tL2CAP_CFG_INFO *p_cfg)
{
    ESP_LOGI(PS3_TAG, "[%s] l2cap_cid: 0x%02x\n  p_cfg->result: %d\n  p_cfg->mtu_present: %d\n  p_cfg->mtu: %d", __func__, l2cap_cid, p_cfg->result, p_cfg->mtu_present, p_cfg->mtu );

    p_cfg->result = L2CAP_CFG_OK;

    L2CA_ConfigRsp(l2cap_cid, p_cfg);
}


/*******************************************************************************
**
** Function         ps3_l2cap_disconnect_ind_cback
**
** Description      This is the L2CAP disconnect indication callback function.
**
**
** Returns          void
**
*******************************************************************************/
void ps3_l2cap_disconnect_ind_cback(uint16_t l2cap_cid, bool ack_needed)
{
    ESP_LOGI(PS3_TAG, "[%s] l2cap_cid: 0x%02x\n  ack_needed: %d", __func__, l2cap_cid, ack_needed );
}


/*******************************************************************************
**
** Function         ps3_l2cap_disconnect_cfm_cback
**
** Description      This is the L2CAP disconnect confirm callback function.
**
**
** Returns          void
**
*******************************************************************************/
static void ps3_l2cap_disconnect_cfm_cback(uint16_t l2cap_cid, uint16_t result)
{
    ESP_LOGI(PS3_TAG, "[%s] l2cap_cid: 0x%02x\n  result: %d", __func__, l2cap_cid, result );
}


/*******************************************************************************
**
** Function         ps3_l2cap_data_ind_cback
**
** Description      This is the L2CAP data indication callback function.
**
**
** Returns          void
**
*******************************************************************************/
static void ps3_l2cap_data_ind_cback(uint16_t l2cap_cid, BT_HDR *p_buf)
{
    if ( p_buf->len > 2 )
    {
        ps3_parse_packet( p_buf->data );
    }

    osi_free( p_buf );
}


/*******************************************************************************
**
** Function         ps3_l2cap_congest_cback
**
** Description      This is the L2CAP congestion callback function.
**
** Returns          void
**
*******************************************************************************/
static void ps3_l2cap_congest_cback (uint16_t l2cap_cid, bool congested)
{
    ESP_LOGI(PS3_TAG, "[%s] l2cap_cid: 0x%02x\n  congested: %d", __func__, l2cap_cid, congested );
}

#else

static void l2cap_read_handle(void * param){
    int size = 0;
    int fd = (int)param;
    uint8_t *l2cap_rd_data = NULL;

    l2cap_rd_data = malloc(L2CAP_DATA_LEN);
    if (!l2cap_rd_data) {
        ESP_LOGE(L2CAP_TAG, "malloc l2cap_rd_data failed, fd:%d", fd);
        goto done;
    }

    do {
        /* The frequency of calling this function also limits the speed at which the peer device can send data. */
        size = read(fd, l2cap_rd_data, L2CAP_DATA_LEN);
        if (size < 0) {
            break;
        } else if (size == 0) {
            /* There is no data, retry after 500 ms */
            vTaskDelay(pdMS_TO_TICKS(500));
        } else {
            ESP_LOGI(L2CAP_TAG, "fd = %d data_len = %d", fd, size);
            /* To avoid task watchdog */
            vTaskDelay(pdMS_TO_TICKS(10));

            if (size > 2) ps3_parse_packet(l2cap_rd_data);
         
        }
    } while (1);
done:
    if (l2cap_rd_data) {
        free(l2cap_rd_data);
    }
    vTaskDelete(NULL);
}

/*******************************************************************************
**
** Function         esp_bt_l2cap_cb
**
** Description      After IDF 5.0, all API calls related to L2CAP trigger 
**                  corresponding events upon completion. This function 
**                  serves as the unified entry point for event handling.
**                  (在IDF5.0后，所有L2CAP相关API执行调用完成均触发相应事件，此函数为统一的事件处理入口)
**
** Returns          void
**
*******************************************************************************/
static void esp_bt_l2cap_cb(esp_bt_l2cap_cb_event_t event, esp_bt_l2cap_cb_param_t *param){
    char bda_str[18] = {0};

    switch (event) {
        case ESP_BT_L2CAP_INIT_EVT:{  /*Due to esp_bt_l2cap_init Trigger*/
            if (param->init.status == ESP_BT_L2CAP_SUCCESS) {
                ESP_LOGI(L2CAP_TAG, "ESP_BT_L2CAP_INIT_EVT: status:%d", param->init.status);
                /* register VFS(虚拟文件系统). Only supports write, read and close.*/
                esp_bt_l2cap_vfs_register();
            } else {
                ESP_LOGE(L2CAP_TAG, "ESP_BT_L2CAP_INIT_EVT: status:%d", param->init.status);
            }
            break;
        }

        case ESP_BT_L2CAP_UNINIT_EVT:{
            ESP_LOGI(L2CAP_TAG, "ESP_BT_L2CAP_UNINIT_EVT: status:%d", param->uninit.status);

            for(int i = 0; i < PS3_L2CAP_channel; i ++){
                channel_Info[i].l2cap_state = -1;
                channel_Info[i].connection_handle = 0; 
                if (channel_Info[i].rd_task_handle != NULL) {
                    vTaskDelete(channel_Info[i].rd_task_handle);
                    channel_Info[i].rd_task_handle = NULL;
                } 
                channel_Info[i].fd = 0;     
            }

            break;
        }

        case ESP_BT_L2CAP_OPEN_EVT:{  /*Due to " 'client connection is established' after esp_bt_l2cap_start_srv" Trigger*/
            if (param->open.status == ESP_BT_L2CAP_SUCCESS) {
                ESP_LOGI(L2CAP_TAG, "ESP_BT_L2CAP_OPEN_EVT: status:%d, fd = %d, tx mtu = %"PRIu32", remote_address:%s", param->open.status,
                        param->open.fd, param->open.tx_mtu, bda2str(param->open.rem_bda, bda_str, sizeof(bda_str)));

                for(int i = 0; i < PS3_L2CAP_channel; i ++){
                    if(channel_Info[i].connection_handle == param->open.handle && \
                       channel_Info[i].l2cap_state == ESP_BT_L2CAP_CL_INIT_EVT){

                        int rd_task_StackSize = 4096;   //default size
                        if(channel_Info[i].psm == BT_PSM_HIDC){
                            /*此信道上传数据量少且低频，分配更小的栈空间以节省开销
                              （The data transmitted through this channel is small in volume and occurs infrequently. Therefore, a smaller 
                               stack space is allocated to reduce costs.)*/
                            rd_task_StackSize = 2048;
                        } 
                        else if(channel_Info[i].psm == BT_PSM_HIDI) ps3Enable();

                        channel_Info[i].l2cap_state = ESP_BT_L2CAP_OPEN_EVT;
                        channel_Info[i].fd = param->open.fd;
                        /*为每个L2CAP_channel数据接收单独创建TASK(Create a separate task for data reception of each L2CAP channel)*/
                        xTaskCreate(l2cap_read_handle, channel_Info[i].name, rd_task_StackSize, (void *) param->open.fd, 5, &channel_Info[i].rd_task_handle);
                        break;
                    }
                }   

            } else {
                ESP_LOGE(L2CAP_TAG, "ESP_BT_L2CAP_OPEN_EVT: status:%d", param->open.status);
            }
            break;
        }

        case ESP_BT_L2CAP_CLOSE_EVT:{  /*Due to " 'client connection has been disconnected' after esp_bt_l2cap_start_srv" Trigger*/
            ESP_LOGI(L2CAP_TAG, "ESP_BT_L2CAP_CLOSE_EVT: status:%d", param->close.status);
            
            for(int i = 0; i < PS3_L2CAP_channel; i ++){
                if(channel_Info[i].connection_handle == param->close.handle && \
                    channel_Info[i].l2cap_state == ESP_BT_L2CAP_OPEN_EVT){

                    channel_Info[i].l2cap_state = ESP_BT_L2CAP_CLOSE_EVT;
                    if (channel_Info[i].rd_task_handle != NULL) {
                        vTaskDelete(channel_Info[i].rd_task_handle);
                        channel_Info[i].rd_task_handle = NULL;
                    } 
                    if (channel_Info[i].fd != 0) {
                        close(channel_Info[i].fd);
                        channel_Info[i].fd = 0;
                    }

                    /*temporary fix measures of expressif Official bug*/
                     esp_bt_l2cap_start_srv(channel_Info[i].security_id, channel_Info[i].psm); // bug, need to do fix
                     break;
                }
            } 

            break;
        }

        case ESP_BT_L2CAP_CL_INIT_EVT:{  /*Due to " 'client initiated a connection' after esp_bt_l2cap_start_srv" Trigger*/
            ESP_LOGI(L2CAP_TAG, "ESP_BT_L2CAP_CL_INIT_EVT: status:%d", param->cl_init.status);

            for(int i = 0; i < PS3_L2CAP_channel; i ++){
                if(channel_Info[i].connection_handle == param->cl_init.handle && \
                   channel_Info[i].l2cap_state == ESP_BT_L2CAP_START_EVT){

                    channel_Info[i].l2cap_state = ESP_BT_L2CAP_CL_INIT_EVT;
                    break;
                }
            }

            break;
        }

        case ESP_BT_L2CAP_START_EVT:{  /*Due to " 'server is started successfully' after esp_bt_l2cap_start_srv" Trigger*/
            if (param->start.status == ESP_BT_L2CAP_SUCCESS) {
                ESP_LOGI(L2CAP_TAG, "ESP_BT_L2CAP_START_EVT: status:%d, hdl:0x%"PRIx32", sec_id:0x%x",
                    param->start.status, param->start.handle, param->start.sec_id);

                /*START_EVT 回调顺序与调用 esp_bt_l2cap_start_srv 顺序一致
                 (The callback sequence of START_EVT is consistent with the sequence of calling esp_bt_l2cap_start_srv.)*/
                static uint8_t L2CAP_channel_started = 0; 
                if(L2CAP_channel_started == PS3_L2CAP_channel){
                    /*已完成全部所需信道第一次的L2CAP服务器初始化，后续信道断连重启L2CAP服务器进入此分支
                      (All required channels have completed their first L2CAP server initialization; subsequent channel disconnections 
                       and restarts of the L2CAP server will enter this branch.)*/
                    for(int i = 0; i < PS3_L2CAP_channel; i ++){
                        if(channel_Info[i].l2cap_state ==ESP_BT_L2CAP_CLOSE_EVT){
                            channel_Info[i].connection_handle = param->start.handle;
                            channel_Info[i].l2cap_state = ESP_BT_L2CAP_START_EVT;                            
                            break;
                        }
                    }

                }else{
                    /*全部所需信道第一次的L2CAP服务器初始化未完成，进入此分支
                      (The first L2CAP server initialization for all required channels has not been completed, entering this branch)*/
                    if(L2CAP_channel_started < PS3_L2CAP_channel){
                        channel_Info[L2CAP_channel_started].connection_handle = param->start.handle;
                        channel_Info[L2CAP_channel_started].l2cap_state = ESP_BT_L2CAP_START_EVT;
                        L2CAP_channel_started ++;                    
                    }                    
                }

            } else {
                ESP_LOGE(L2CAP_TAG, "ESP_BT_L2CAP_START_EVT: status:%d", param->start.status);
            }
            break;
        }

        case ESP_BT_L2CAP_SRV_STOP_EVT:{  /*Due to esp_bt_l2cap_stop_srv Trigger*/
            ESP_LOGI(L2CAP_TAG, "ESP_BT_L2CAP_SRV_STOP_EVT: status:%d, psm = 0x%x", param->srv_stop.status, param->srv_stop.psm);

            for(int i = 0; i < PS3_L2CAP_channel; i ++){
                if(channel_Info[i].psm == param->srv_stop.psm){
                    channel_Info[i].l2cap_state = -1;
                    channel_Info[i].connection_handle = 0;
                    if (channel_Info[i].rd_task_handle != NULL) {
                        vTaskDelete(channel_Info[i].rd_task_handle);
                        channel_Info[i].rd_task_handle = NULL;
                    }    
 
                    break;
                }
            } 

            break;
        }

        case ESP_BT_L2CAP_VFS_REGISTER_EVT:{  /*Due to esp_bt_l2cap_vfs_register Trigger*/
            if (param->vfs_register.status == ESP_BT_L2CAP_SUCCESS) {
                ESP_LOGI(L2CAP_TAG, "ESP_BT_L2CAP_VFS_REGISTER_EVT: status:%d", param->vfs_register.status);
                /*Create an L2CAP server and start listening for connection requests.(创建L2CAP服务器并开始监听连接请求)*/
                /*ESP_BT_L2CAP_SEC_NONE(0): No security*/
                for(int i = 0; i < PS3_L2CAP_channel; i ++){
                    esp_bt_l2cap_start_srv(channel_Info[i].security_id, channel_Info[i].psm);
                }
                
            } else {
                ESP_LOGE(L2CAP_TAG, "ESP_BT_L2CAP_VFS_REGISTER_EVT: status:%d", param->vfs_register.status);
            }
            break;
        }

        default:
            break;
    }
    return;
}

#endif