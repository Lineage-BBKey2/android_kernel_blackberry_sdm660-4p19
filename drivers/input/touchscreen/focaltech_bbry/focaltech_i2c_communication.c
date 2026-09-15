/*
 *
 * FocalTech TouchScreen driver.
 * 
 * Copyright (c) 2010-2016, FocalTech Systems, Ltd., all rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

 /************************************************************************
*
* File Name: focaltech_apk_node.c
*
* Author:	  Xu YF & ZR,  Software Department, FocalTech
*
* Created: 2016-04-08
*   
* Modify:
*
* Abstract: i2c communication with TP
*
************************************************************************/

/*******************************************************************************
* Included header files
*******************************************************************************/
#include "focaltech_comm.h"

/*******************************************************************************
* Private constant and macro definitions using #define
*******************************************************************************/
#define FOCALTECH_I2C_COMMUNICATION_INFO  "File Version of  focaltech_i2c_communication.c:  V1.0.0 2016-04-08"

/*i2c flag for write*/
#define I2C_M_WRITE    0

/*dma declare, allocate and release*/
#define FTS_DMA_BUFF_SIZE     128
#define FTS_DMA_EN      0//0:disable, 1:enable 




/*******************************************************************************
* Private enumerations, structures and unions using typedef
*******************************************************************************/


/*******************************************************************************
* Static variables
*******************************************************************************/
/*如果要定义一个静态mutex型变量，应该使用DEFINE_MUTEX*/
static DEFINE_MUTEX(i2c_rw_access);

#if FTS_DMA_EN
static u8 *g_dma_buff_va = NULL;
static u8 *g_dma_buff_pa = NULL;
#endif

/*******************************************************************************
* Global variable or extern global variabls/functions
*******************************************************************************/

/*******************************************************************************
* Static function prototypes
*******************************************************************************/
static void msg_dma_alloct(void);
static void msg_dma_release(void);
/*******************************************************************************
* functions body
*******************************************************************************/

/*******************************************************************************
*  Name: fts_i2c_read
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
int fts_i2c_read_universal(struct i2c_client *client, char *writebuf,
			   int writelen, char *readbuf, int readlen)
{
	int ret;
	int expected_msgs;
	u8 *dma_writebuf = NULL;
	u8 *dma_readbuf;
	struct i2c_msg msgs[2];

	if (!client || !client->adapter)
		return -ENODEV;

	if (writelen < 0 || readlen < 0 ||
	    (writelen > 0 && !writebuf) ||
	    (readlen > 0 && !readbuf))
		return -EINVAL;

	if (readlen == 0)
		return 0;

	/*
	 * i2c-msm-v2 may DMA-map both messages when the complete transfer is
	 * large enough for DMA mode. FocalTech callers may provide stack-backed
	 * buffers, which are not guaranteed to be DMA-safe, so use kmalloc-backed
	 * bounce buffers before handing the messages to the adapter.
	 */
	if (writelen > 0) {
		dma_writebuf = kmemdup(writebuf, writelen, GFP_KERNEL);
		if (!dma_writebuf)
			return -ENOMEM;
	}

	dma_readbuf = kmalloc(readlen, GFP_KERNEL);
	if (!dma_readbuf) {
		kfree(dma_writebuf);
		return -ENOMEM;
	}

	memset(msgs, 0, sizeof(msgs));

	if (writelen > 0) {
		msgs[0].addr = client->addr;
		msgs[0].flags = I2C_M_WRITE | I2C_M_DMA_SAFE;
		msgs[0].len = writelen;
		msgs[0].buf = dma_writebuf;

		msgs[1].addr = client->addr;
		msgs[1].flags = I2C_M_RD | I2C_M_DMA_SAFE;
		msgs[1].len = readlen;
		msgs[1].buf = dma_readbuf;

		expected_msgs = 2;
	} else {
		msgs[0].addr = client->addr;
		msgs[0].flags = I2C_M_RD | I2C_M_DMA_SAFE;
		msgs[0].len = readlen;
		msgs[0].buf = dma_readbuf;

		expected_msgs = 1;
	}

	mutex_lock(&i2c_rw_access);

	ret = i2c_transfer(client->adapter, msgs, expected_msgs);
	if (ret == expected_msgs) {
		memcpy(readbuf, dma_readbuf, readlen);
	} else if (ret < 0) {
		FTS_COMMON_DBG("i2c read error.");
	} else {
		ret = -EIO;
	}

	mutex_unlock(&i2c_rw_access);

	kfree(dma_readbuf);
	kfree(dma_writebuf);

	return ret;
}

/*******************************************************************************
*  Name: fts_i2c_write
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
int fts_i2c_write_universal(struct i2c_client *client, char *writebuf,
			    int writelen)
{
	int ret;
	u8 *dma_writebuf;
	struct i2c_msg msg;

	if (!client || !client->adapter)
		return -ENODEV;

	if (writelen < 0 || (writelen > 0 && !writebuf))
		return -EINVAL;

	if (writelen == 0)
		return 0;

	dma_writebuf = kmemdup(writebuf, writelen, GFP_KERNEL);
	if (!dma_writebuf)
		return -ENOMEM;

	memset(&msg, 0, sizeof(msg));
	msg.addr = client->addr;
	msg.flags = I2C_M_WRITE | I2C_M_DMA_SAFE;
	msg.len = writelen;
	msg.buf = dma_writebuf;

	mutex_lock(&i2c_rw_access);

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0)
		FTS_COMMON_DBG("i2c write error.");
	else if (ret != 1)
		ret = -EIO;

	mutex_unlock(&i2c_rw_access);

	kfree(dma_writebuf);

	return ret;
}


/************************************************************************
* Name: fts_i2c_read
* Brief: i2c read
* Input: i2c info, write buf, write len, read buf, read len
* Output: get data in the 3rd buf
* Return: fail <0
***********************************************************************/
int fts_i2c_read_dma(struct i2c_client *client, char *writebuf,int writelen, char *readbuf, int readlen)
{
	int ret=0;

#if FTS_DMA_EN
	// for DMA I2c transfer
	
	mutex_lock(&i2c_rw_access);
	
	if(writelen!=0)
	{
		//DMA Write
		memcpy(g_dma_buff_va, writebuf, writelen);
		client->addr = client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG;
		if((ret=i2c_master_send(client, (unsigned char *)g_dma_buff_pa, writelen))!=writelen)
			//dev_err(&client->dev, "###%s i2c write len=%x,buffaddr=%x\n", __func__,ret,*g_dma_buff_pa);
			printk("i2c write failed\n");
		client->addr = client->addr & I2C_MASK_FLAG &(~ I2C_DMA_FLAG);
	}

	//DMA Read 

	if(readlen!=0)

	{
		client->addr = client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG;

		ret = i2c_master_recv(client, (unsigned char *)g_dma_buff_pa, readlen);

		memcpy(readbuf, g_dma_buff_va, readlen);

		client->addr = client->addr & I2C_MASK_FLAG &(~ I2C_DMA_FLAG);
	}
	
	mutex_unlock(&i2c_rw_access);
#endif

	return ret;

}

/************************************************************************
* Name: fts_i2c_write
* Brief: i2c write
* Input: i2c info, write buf, write len
* Output: no
* Return: fail <0
***********************************************************************/
int fts_i2c_write_dma(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret = 0;
	
#if FTS_DMA_EN
	mutex_lock(&i2c_rw_access);
	
 	//client->addr = client->addr & I2C_MASK_FLAG;

	//ret = i2c_master_send(client, writebuf, writelen);
	memcpy(g_dma_buff_va, writebuf, writelen);
	
	client->addr = client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG;
	if((ret=i2c_master_send(client, (unsigned char *)g_dma_buff_pa, writelen))!=writelen)
		//dev_err(&client->dev, "###%s i2c write len=%x,buffaddr=%x\n", __func__,ret,*g_dma_buff_pa);
		printk("i2c write failed\n");
	client->addr = client->addr & I2C_MASK_FLAG &(~ I2C_DMA_FLAG);
		
	mutex_unlock(&i2c_rw_access);
#endif

	return ret;

}



static void msg_dma_alloct(void)
{
#if FTS_DMA_EN
	g_dma_buff_va = (u8 *)dma_alloc_coherent(NULL, FTS_DMA_BUFF_SIZE, &g_dma_buff_pa, GFP_KERNEL);//DMA size 4096 for customer
	if(!g_dma_buff_va)
	{
	        FTS_COMMON_DBG("[DMA][Error] Allocate DMA I2C Buffer failed!\n");
	}
	else
	{
		FTS_COMMON_DBG("[DMA] Allocate DMA I2C Buffer succeeded!\n");
	}
#endif
	return;
}
static void msg_dma_release(void)
{
#if FTS_DMA_EN
	if(g_dma_buff_va)
	{
	     	dma_free_coherent(NULL, FTS_DMA_BUFF_SIZE, g_dma_buff_va, g_dma_buff_pa);
	        g_dma_buff_va = NULL;
	        g_dma_buff_pa = NULL;
		FTS_COMMON_DBG("[DMA][release] Allocate DMA I2C Buffer release!\n");
	 }
#endif

	return;
}

int fts_i2c_communication_init(void)
{	
	FTS_COMMON_DBG("[focal] %s ",  FOCALTECH_I2C_COMMUNICATION_INFO);	//show version
	FTS_COMMON_DBG("");//default print: current function name and line number
	msg_dma_alloct();
	
	return 0;
}
/************************************************************************
* Name: fts_sysfs_exit
* Brief:  remove sysfs
* Input: i2c info
* Output: no
* Return: no
***********************************************************************/
int fts_i2c_communication_exit(void)
{
	FTS_COMMON_DBG("");//default print: current function name and line number
	msg_dma_release();	
	
	return 0;
}

