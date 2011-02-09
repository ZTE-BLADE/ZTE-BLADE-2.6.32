

#include <linux/module.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/timer.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <asm/io.h>
#include <linux/earlysuspend.h>
#include <asm/system.h>
#include <asm/mach-types.h>
#include <asm/uaccess.h>
#include <asm/gpio.h>
#include <mach/fm_si4708.h>

#define   	FM_DBG(x...)   	printk(KERN_DEBUG"[FM]"x)
#define   	FM_INFO(x...)   	printk(KERN_INFO"[FM]"x)
#define 	FYA_TAG        "[FYA@FM_SI4708]"

struct fm_dev {
	int   initialized;   			/* device opend times */
    	char   band;				/* cached band */		
    	char   track;				/* cached track */
    	char   status_stereo;		/* cached stereo status */
	char  space;				/* cached space */
   	int    frequency; 			/* cached frequency */
    	int    channel;			/* cached channel */
	char  seek_direction;		/* cached seek direction */
    	char  vol;         			/* cached vol */          	  		      											                	 						                                   											                	  		
    	char regs[32];             		/* buffer for reading */	 		 			
	struct i2c_client *client;
	unsigned char* buffer;
	unsigned int read_index;
	unsigned int write_index;
	spinlock_t lock;
};

static struct fm_dev *fm_si4708_dev;
static unsigned int buffer_size = 50;
static int si4708_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int si4708_remove(struct i2c_client *client);

static struct timer_list rds_timer;
static int rds_enabled = 1;
static int rds_read_index = 0;
static unsigned char rds_read_buffer[8];

static int si4708_regs_read(struct i2c_client * client, char * buf, int count)
{
	int	ret = 0;

	if (count != i2c_master_recv(client, buf, count)) 
		ret = -EIO;

	return ret;
}


static int si4708_regs_write(struct i2c_client * client,const char * buf, int count)
{
	int	ret = 0;

	if (count != i2c_master_send(client, buf, count)) 
		ret = -EIO;

	return ret;
}


/**********************************************************************/

#define int_to_char(m)  ((char)(m&0xFF))


static int si4708_WR(char *in_data, int num, int isInit)
{
	int res;

	res = si4708_regs_write(fm_si4708_dev->client, in_data, num);
   	if (res < 0)
		fm_si4708_dev->initialized = 0;
	 
   	/* from powerdown to powerup*/
	if(isInit)
		mdelay(SI4708_POWERUP_DELAY);
	
	return res;
}


/**********************************************************************/
/*three modes :	init,	wakeup and standby*/
static char fm_init[] = {0x40,0x01,0x00,0x00,0xC0,0x04,0x0f,0x10};
static char fm_wakeup[] = {0x40,0x01};
static char fm_standby[] = {0x00,0x41};

#define si4708_init2normal(void)	si4708_WR(fm_init,sizeof(fm_init),1)
#define si4708_standby2normal(void)	si4708_WR(fm_wakeup,sizeof(fm_wakeup),0)
#define si4708_normal2standby(void)	si4708_WR(fm_standby,sizeof(fm_standby),0)


static int si4708_read_all_regs(void)
{
	int res = 0;
	
    	res = si4708_regs_read(fm_si4708_dev->client, &fm_si4708_dev->regs[FM_REG_RD_START], sizeof(fm_si4708_dev->regs));
    	if (res < 0)
		fm_si4708_dev->initialized = 0;

	return res;
}


static int si4708_get_vol(void)
{
	fm_si4708_dev->vol = fm_si4708_dev->regs[FM_REG_SYSCONFIG2+1] & 0X0F;
	return fm_si4708_dev->vol;
}


static int si4708_set_vol(char vol)
{
	int res;
	int vol_value;

       fm_si4708_dev->vol = vol & 0xF;
       vol_value = (int)fm_si4708_dev->vol ;
	   
    	if(vol_value < FM_VOL_MUTE)
		fm_si4708_dev->vol = int_to_char(FM_VOL_MUTE);
		
	if(vol_value > FM_VOL_MAX) 
   	       fm_si4708_dev->vol = int_to_char(FM_VOL_MAX);
		
    	fm_si4708_dev->regs[FM_REG_SYSCONFIG2+1] &= ~REG_VOL_MASK; 
	fm_si4708_dev->regs[FM_REG_SYSCONFIG2+1] |=  fm_si4708_dev->vol; 
	/*write  vol  to register*/
    	res = si4708_regs_write(fm_si4708_dev->client, &fm_si4708_dev->regs[FM_REG_WR_START], 8);
	if(res < 0)
		fm_si4708_dev->initialized = 0;

    	return res;
}


static int si4708_get_band(void)
{  
	fm_si4708_dev->band = fm_si4708_dev->regs[FM_REG_SYSCONFIG2+1] & 0XC0;
	return fm_si4708_dev->band;
}


static int si4708_set_band(char band)
{
	int res = 0;
	int band_value;
	
       fm_si4708_dev->band = band & 0x3;
       band_value =(int)fm_si4708_dev->band;
	   
    	if(band_value < FM_BAND_GENERIC )
		fm_si4708_dev->band = FM_BAND_GENERIC;
		
	if(band_value > FM_BAND_JAPAN) 
   	       fm_si4708_dev->band = FM_BAND_JAPAN;
	   
	fm_si4708_dev->regs[FM_REG_SYSCONFIG2+1] &= ~REG_BAND_MASK; 
	fm_si4708_dev->regs[FM_REG_SYSCONFIG2+1] |= fm_si4708_dev->band <<6; 

	
	/*write  band  to register*/
    	res = si4708_regs_write(fm_si4708_dev->client, &fm_si4708_dev->regs[FM_REG_WR_START], 8);
	if(res < 0)
		fm_si4708_dev->initialized = 0;
		
    	return res;
}


static int si4708_get_space(void)
{
	fm_si4708_dev->space = fm_si4708_dev->regs[FM_REG_SYSCONFIG2+1] & 0X30;
	return fm_si4708_dev->space;
}


static int si4708_set_space(char space)
{
	int res = 0;
	int space_value;
	
       fm_si4708_dev->space = space & 0x3;
	space_value = fm_si4708_dev->space;
	   
    	if(space_value < FM_SPACE_200K )
		fm_si4708_dev->space = FM_SPACE_200K;
		
	if(space_value > FM_SPACE_50K) 
   	       fm_si4708_dev->space = FM_SPACE_50K;
	   
	fm_si4708_dev->regs[FM_REG_SYSCONFIG2+1] &= ~REG_SPACE_MASK; 
	fm_si4708_dev->regs[FM_REG_SYSCONFIG2+1] |= fm_si4708_dev->space <<4; 
	
	/*write  space  to register*/
    	res = si4708_regs_write(fm_si4708_dev->client, &fm_si4708_dev->regs[FM_REG_WR_START], 8);
	if(res < 0)
		fm_si4708_dev->initialized = 0;
		
    	return res;
}


static int si4708_get_track(void) 
{ 
 	fm_si4708_dev->track = fm_si4708_dev->regs[FM_REG_POWERCFG] & 0X20;
	return fm_si4708_dev->track;
}


static int si4708_set_track(char track)
{
	int res = 0;

       fm_si4708_dev->track = track & 0x1;
	   
	fm_si4708_dev->regs[FM_REG_POWERCFG] &= ~REG_TRACK_MASK; 
	fm_si4708_dev->regs[FM_REG_POWERCFG] |= fm_si4708_dev->track <<5; 
	
	/*write  vol  to register*/
    	res = si4708_regs_write(fm_si4708_dev->client, &fm_si4708_dev->regs[FM_REG_WR_START], 1);
	if(res < 0)
			fm_si4708_dev->initialized = 0;
		
  return res;
}



static int channel2freq(int channel)
{
	int	freq = 0;
	int	space;
	int	space_status = si4708_get_space();

	switch(space_status){
		case FM_SPACE_200K:
			space = 20;
			break;
		case FM_SPACE_100K:
			space = 10;
			break;
		case FM_SPACE_50K:
			space = 5;
			break;
		default:
		       space = 10;
	}

	if(fm_si4708_dev->band == FM_BAND_GENERIC)
		freq = space * channel + 8750;/* US &  EUROPE 87.50MHZ */
	else
		freq = space * channel + 7600;/* JAPAN 76.00MHZ */

	return freq;
	
}


static int freq2channel(int frequency)
{
	int   space;
	int   channel;
	int 	space_status = si4708_get_space();

	switch(space_status){
		case FM_SPACE_200K:
			space = 20;
			break;
		case FM_SPACE_100K:
			space = 10;
			break;
		case FM_SPACE_50K:
			space = 5;
			break;
		default:
		       space = 10;
	}
	
	if(fm_si4708_dev->band == FM_BAND_GENERIC)
		channel = (frequency - 8750)/space;
	else  /*JAPAN BAND*/
		channel = (frequency -7600)/space;
	
       return channel;
}


static int si4708_get_frequency(void)
{
	int res = 0;
	int high,low,channel;

	res = si4708_regs_read(fm_si4708_dev->client, 
			&fm_si4708_dev->regs[FM_REG_RD_START], 4); 
	if(res <0)
	{
		fm_si4708_dev->initialized = 0;
    		return res;
	}

	high = ((int)(fm_si4708_dev->regs[FM_REG_READCHAN] & 0x03))<<8;
	low = (int)fm_si4708_dev->regs[FM_REG_READCHAN+1];

	channel = high + low;

	return channel2freq(channel);

}
static int si4708_set_frequency(int frequency)
{
  	int res = 0;

  	res = si4708_regs_read(fm_si4708_dev->client, &fm_si4708_dev->regs[FM_REG_RD_START], 4); 
	if(res <0)
	{
		fm_si4708_dev->initialized = 0;
    		return res;
	}
	if( (fm_si4708_dev->regs[FM_REG_STATUSRSSI] & REG_STC_MASK) == REG_STC_MASK)
		return -EBUSY;
	
       fm_si4708_dev->frequency = frequency;
	if(fm_si4708_dev->band == FM_BAND_GENERIC)
	{
		if(frequency > 10800)
			fm_si4708_dev->frequency = 10800;
		if(frequency < 8750)
			fm_si4708_dev->frequency = 8750;
	}
	else if(fm_si4708_dev->band == FM_BAND_JAPAN_WIDE)
	{
		if(frequency > 10800)
			fm_si4708_dev->frequency = 10800;
		if(frequency < 7600)
			fm_si4708_dev->frequency = 7600;	
	}
	else
	{
		if(frequency > 9000)
			fm_si4708_dev->frequency = 9000;
		if(frequency < 7600)
			fm_si4708_dev->frequency = 7600;	
	}
	
       fm_si4708_dev->channel = freq2channel(frequency);
	
    	fm_si4708_dev->regs[FM_REG_CHANNEL+1] = int_to_char(fm_si4708_dev->channel);
	fm_si4708_dev->regs[FM_REG_CHANNEL] = REG_TUNE_MASK|int_to_char(fm_si4708_dev->channel >>8);

    	res = si4708_regs_write(fm_si4708_dev->client, &fm_si4708_dev->regs[FM_REG_WR_START], 4); //start tune.
	if(res <0)
	{
		fm_si4708_dev->initialized = 0;
    		return res;
	}

	mdelay(SI4708_TUNE_DELAY);    /*seek time*/
	   
	/*check STC status*/
       res = si4708_regs_read(fm_si4708_dev->client, &fm_si4708_dev->regs[FM_REG_RD_START], 4); 
	if(res <0)
	{
		fm_si4708_dev->initialized = 0;
    		return res;
	}

       //no matter what's the STC status, we should shold stop tunning here.
	fm_si4708_dev->regs[FM_REG_CHANNEL] &= ~REG_TUNE_MASK;
    	res = si4708_regs_write(fm_si4708_dev->client, &fm_si4708_dev->regs[FM_REG_WR_START], 3); 
	if(res < 0)
	{
	       fm_si4708_dev->initialized = 0;
		return res;
	}

	/*check whether tune complete*/
	if( REG_STC_MASK & fm_si4708_dev->regs[FM_REG_STATUSRSSI])
	{	
		//clear STC
		fm_si4708_dev->regs[FM_REG_POWERCFG] = 0x40;
		fm_si4708_dev->regs[FM_REG_CHANNEL] &= ~REG_TUNE_MASK; 
		si4708_regs_write(fm_si4708_dev->client, &fm_si4708_dev->regs[FM_REG_POWERCFG], 3); 

		return res;
	 }
	else
	{
		//clear STC
		fm_si4708_dev->regs[FM_REG_POWERCFG] = 0x40;
		fm_si4708_dev->regs[FM_REG_CHANNEL] &= ~REG_TUNE_MASK; 
		si4708_regs_write(fm_si4708_dev->client, &fm_si4708_dev->regs[FM_REG_POWERCFG], 3); 

		return -EFAULT;
	}
	
}


static int si4708_seek(int dir, int *p_frequency)
{
    	int res = 0;
	int seeked_channel;

       res = si4708_regs_read(fm_si4708_dev->client, &fm_si4708_dev->regs[FM_REG_RD_START], 24);
	if(res <0)
	{
		fm_si4708_dev->initialized = 0;
    		return res;
	}
	if( (fm_si4708_dev->regs[FM_REG_STATUSRSSI] & REG_STC_MASK) == REG_STC_MASK)
		return -EBUSY;
	if(dir == SEEKDOWN)
	{
		fm_si4708_dev->regs[FM_REG_POWERCFG] &= ~REG_SEEKUP_MASK;
		fm_si4708_dev->regs[FM_REG_POWERCFG] |= REG_SEEK_MASK;
	}
	if(dir == SEEKUP)
    		 fm_si4708_dev->regs[FM_REG_POWERCFG] |= REG_SEEKUP_MASK|REG_SEEK_MASK; 

	fm_si4708_dev->regs[FM_REG_POWERCFG] &= ~REG_SKMODE_MASK;
   	res = si4708_regs_write(fm_si4708_dev->client, &fm_si4708_dev->regs[FM_REG_WR_START], 1);//start seek
	if(res <0 )
	{
		 fm_si4708_dev->initialized = 0;
    		 return res;
	}
	
	mdelay(SI4708_SEEK_TUNE_DELAY);    /*seek time*/
	   
	/*check it is whether seek complete*/
       res = si4708_regs_read(fm_si4708_dev->client, &fm_si4708_dev->regs[FM_REG_RD_START], 4); //read statusrssi
	if(res <0)
	{
		fm_si4708_dev->initialized = 0;
    		return res;
	}
	fm_si4708_dev->regs[FM_REG_CHANNEL] &= ~REG_TUNE_MASK;
    	res = si4708_regs_write(fm_si4708_dev->client, &fm_si4708_dev->regs[FM_REG_WR_START], 3); 
	if(res < 0)
	{
	       fm_si4708_dev->initialized = 0;
		return res;
	}
	if( REG_STC_MASK & fm_si4708_dev->regs[FM_REG_STATUSRSSI])
	{	
		if( REG_SF_MASK & fm_si4708_dev->regs[FM_REG_STATUSRSSI])
		{
			*p_frequency = channel2freq(fm_si4708_dev->channel);	
		}
		else
		{
			seeked_channel = (int) fm_si4708_dev->regs[FM_REG_READCHAN+1]+(int)((fm_si4708_dev->regs[FM_REG_READCHAN]& 0x3) <<8);
			*p_frequency = channel2freq(seeked_channel);
		}
		//clear STC and SF
		fm_si4708_dev->regs[FM_REG_POWERCFG] = 0x40;
		fm_si4708_dev->regs[FM_REG_CHANNEL] &= ~REG_TUNE_MASK; 
		si4708_regs_write(fm_si4708_dev->client, &fm_si4708_dev->regs[FM_REG_POWERCFG], 3); 

		 return res;
	 }
	else
	{	
		//clear STC and SF
		fm_si4708_dev->regs[FM_REG_POWERCFG] = 0x40;
		fm_si4708_dev->regs[FM_REG_CHANNEL] &= ~REG_TUNE_MASK; 
		si4708_regs_write(fm_si4708_dev->client, &fm_si4708_dev->regs[FM_REG_POWERCFG], 3); 
		
		return -EBUSY;
	}
}
extern int snd_hph_amp_ctl(uint32_t on);



static void setRDSEnabled(int enabled){
    int ret;
    ret = 0;
    rds_enabled = enabled;
    fm_si4708_dev->regs[FM_REG_SYSCONFIG1] &= ~0x90;
    fm_si4708_dev->regs[FM_REG_SYSCONFIG1+1] &= ~0x04;
    if (rds_enabled == 1) {	//switch on rds
	fm_si4708_dev->regs[FM_REG_SYSCONFIG1] |= 0x90;
        fm_si4708_dev->regs[FM_REG_SYSCONFIG1+1] |= 0x04;
    }
    ret = si4708_regs_write(fm_si4708_dev->client, &fm_si4708_dev->regs[FM_REG_WR_START], 8);
    if(ret < 0)
	fm_si4708_dev->initialized = 0;
    if (rds_enabled == 1){
        mod_timer(&rds_timer, jiffies + msecs_to_jiffies(100));
    }
}



static int si4708_get_rds(void){
    int avail = 0;
    int i=0;
    
    if (rds_read_index == 0){
	if (fm_si4708_dev->read_index == fm_si4708_dev->write_index) return 0;
	memcpy(&rds_read_buffer,&fm_si4708_dev->buffer[fm_si4708_dev->read_index],8);
	fm_si4708_dev->read_index+=8;
	if (rds_enabled==2) setRDSEnabled(1);
	if (fm_si4708_dev->read_index >= buffer_size*8) fm_si4708_dev->read_index = 0;
    }
    i=rds_read_index<<24 | rds_read_buffer[rds_read_index*3]<<16 | rds_read_buffer[rds_read_index*3+1]<<8;
    if (rds_read_index != 2) {
	i|=rds_read_buffer[rds_read_index*3+2];
	rds_read_index++;
    }
    else {
	if (fm_si4708_dev->read_index != fm_si4708_dev->write_index) avail = 1;
	i|=avail;
	rds_read_index = 0;
    }
    return i;
}

static void resetRDSBuffer(void){
	fm_si4708_dev->read_index = 0;
	fm_si4708_dev->write_index = 0;
}


static unsigned int readRDS(void){
        int res = si4708_regs_read(fm_si4708_dev->client, &fm_si4708_dev->regs[FM_REG_STATUSRSSI], 12);
	if(res <0)
	{
		fm_si4708_dev->initialized = 0;
		return res;
	}
//		printk("RDS data %d, %d\n",fm_si4708_dev->read_index,fm_si4708_dev->write_index);
	    if ((fm_si4708_dev->regs[0] & 0x80) == 0x80){
		if ( ((fm_si4708_dev->regs[0] & 0x06) <=1) &&
		((fm_si4708_dev->regs[2] & 0xC0) <=1) &&
		((fm_si4708_dev->regs[2] & 0x30) <=1) &&
		((fm_si4708_dev->regs[2] & 0x0C) <=1) ){
			memcpy(&fm_si4708_dev->buffer[fm_si4708_dev->write_index],&fm_si4708_dev->regs[FM_REG_RDSA],2);
			memcpy(&fm_si4708_dev->buffer[fm_si4708_dev->write_index+2],&fm_si4708_dev->regs[FM_REG_RDSB],2);
			memcpy(&fm_si4708_dev->buffer[fm_si4708_dev->write_index+4],&fm_si4708_dev->regs[FM_REG_RDSC],2);
			memcpy(&fm_si4708_dev->buffer[fm_si4708_dev->write_index+6],&fm_si4708_dev->regs[FM_REG_RDSD],2);
			fm_si4708_dev->write_index += 8;
			if (fm_si4708_dev->write_index >= buffer_size*8){
			    fm_si4708_dev->write_index = 0;
			}
			if (fm_si4708_dev->write_index == fm_si4708_dev->read_index){
			    setRDSEnabled(2);
			    fm_si4708_dev->read_index+=8;
			    if (fm_si4708_dev->read_index >= buffer_size*8){
				fm_si4708_dev->read_index = 0;
			    }
			}
		}
		return 1;
    	    }
	    else{
//		printk("RDS data MISSED\n");
		return 0;
	    }
	    return 0;

}

static void timer_func(unsigned long data){
    unsigned int next = 70+readRDS()*10;
    if (rds_enabled == 1) mod_timer(&rds_timer,jiffies+msecs_to_jiffies(next));
}


static int si4708_open(struct inode *inode, struct file *filp)
{
    	int res;
	int sdio_state;
	
	snd_hph_amp_ctl(0);
    
	if(fm_si4708_dev->initialized)
	{
	    	res = si4708_standby2normal();
	}
	else
       {
		spin_lock_init(&fm_si4708_dev->lock);
		gpio_direction_output(FM_RESET_GPIO,0);
		mdelay(SI4708_RESET_DELAY);
		
		spin_lock_irq(&fm_si4708_dev->lock);
		sdio_state = gpio_get_value(FM_I2C_SDA_GPIO);
		gpio_direction_output(FM_I2C_SDA_GPIO, 0); 
		udelay(2);
		gpio_direction_output(FM_RESET_GPIO,1);
		udelay(2);
		gpio_direction_output(FM_I2C_SDA_GPIO,sdio_state); 
		spin_unlock_irq(&fm_si4708_dev->lock);

		res = si4708_init2normal();
		if( res > 0)
		     fm_si4708_dev->initialized++;
	}

	snd_hph_amp_ctl(1);

       if(res < 0)
	   	return res;

	res = si4708_read_all_regs();
	si4708_get_band();
	si4708_get_vol();
	si4708_get_space();
	si4708_get_track();
	si4708_get_frequency();
	return res < 0 ? res : 0;
}


static int si4708_release(struct inode *inode, struct file *filp)
{
	int res;
	/*set close status,flag = 0*/
	fm_si4708_dev->initialized = 0;
	/* Make sure the device is power down. */
	res = si4708_normal2standby();
	rds_enabled=0;
	return res < 0 ? res : 0;
}


static int si4708_ioctl(struct inode *inode, struct file *filp,
                     unsigned int cmd, unsigned long arg)
{
    	int res = 0;
	int user_data;
	int user_value[2];
    	
    	switch (cmd) 
	{
		case	FM_INIT2NORMAL:
			res = si4708_init2normal(); 
		break;
		
		case	FM_NORMAL2STANDBY:
			setRDSEnabled(0);
			res = si4708_normal2standby(); 
		break;
		
		case	FM_STANDBY2NORMAL:
			setRDSEnabled(1);
			res = si4708_standby2normal();		
		break;
	
		case 	FM_TUNE:
			if (get_user(user_data,(int *)arg))
			{
			     res = -EFAULT;
				goto exit;
		       }
	        	res = si4708_set_frequency(user_data);
	       	  resetRDSBuffer();
	        break;

		 case	 FM_SEEK:
			if (copy_from_user(user_value,(int *)arg, 2*sizeof(int)))
			{
	   		    res = -EFAULT;
			    goto exit;
       		}
		res = si4708_seek(user_value[0], &user_value[1]);
			 if(res < 0)
			 	goto exit;
			 if (copy_to_user((int *)arg,user_value, 2*sizeof(int)))
	       	  {
	            	       res = -EFAULT;
			       goto exit;
	       	  }
	        break;
		
		 case 	FM_GET_VOL:
		if (put_user((int)si4708_get_rds(), (int*)arg))
		{
		    res = -EFAULT;
			goto exit;
		}
	        break;

		 case 	FM_SET_VOL:
			if (get_user(user_data,(int *)arg))
			{
	  		       res = -EFAULT;
				goto exit;
    		       }
	       	 res = si4708_set_vol(int_to_char(user_data)); 
	       	 if (user_data>15){
		    setRDSEnabled(1);
	       	}else{
		    setRDSEnabled(0);
	       	}
	        break;

		 case 	FM_GET_BAND:
	       	 if (put_user((int)si4708_get_band(), (int*)arg))
	       	  {
	            	       res = -EFAULT;
			       goto exit;
	       	  }
		 break;
				
	    	 case 	FM_SET_BAND:	
			if (get_user(user_data,(int *)arg))
			{
	  		       res = -EFAULT;
				goto exit;
    		       }
			res = si4708_set_band(int_to_char(user_data));
	        break;
			 
		 case 	FM_GET_SPACE:
	       	 if (put_user((int)si4708_get_space(), (int*)arg))
	       	  {
	            	       res = -EFAULT;
			       goto exit;
	       	  }
	        break;

		 case 	FM_SET_SPACE:
			if (get_user(user_data,(int *)arg))
			{
	  		       res = -EFAULT;
				goto exit;
    		       }
	        	res = si4708_set_space(int_to_char(user_data));
	        break;

		 case	FM_GET_AUDIOTRACK:
	       	 if (put_user((int)si4708_get_track(), (int *)arg))
	       	  {
	            	       res = -EFAULT;
			       goto exit;
	       	  }
		 break;

		 case	FM_SET_AUDIOTRACK:
			if (get_user(user_data,(int *)arg))
			{
	  		     res = -EFAULT;
				goto exit;
    		       }
			res = si4708_set_track(int_to_char(user_data));
		 break;

		 case	FM_GET_CURRENTFREQ:
	       	 if (put_user(si4708_get_frequency(), (int *)arg))
	       	  {
	            	       res = -EFAULT;
			       goto exit;
	       	  }
		 break;
		 
	       default:
	              return  -ENOTTY;
      }
    exit:		
   return res < 0 ?  res : 0;

}
static struct file_operations si4708_fops = {
	 .owner 	= THIS_MODULE,
        .ioctl =	si4708_ioctl,
        .open = si4708_open,
        .release = si4708_release,
};


static struct miscdevice si4708_device = {
	.minor 	=  MISC_DYNAMIC_MINOR,
	.name 	=  "si4708",
	.fops 	= &si4708_fops,
};



static int si4708_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int res = 0;
	
     	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))	
		return -EIO;

	fm_si4708_dev = kzalloc(sizeof(struct fm_dev), GFP_KERNEL);
	if (!fm_si4708_dev){ 
		res = - ENOMEM;
		goto init_exit;
	}

	fm_si4708_dev->buffer = kmalloc(buffer_size*8,GFP_KERNEL);
	if (!fm_si4708_dev->buffer){
		goto out;
	}
	fm_si4708_dev->read_index=0;
	fm_si4708_dev->write_index=0;
	
	setup_timer(&rds_timer, timer_func, 1234);
	
	
	fm_si4708_dev->initialized = 0;
	res = misc_register(&si4708_device);
	if(res)
		goto out;
	
	
	
	FM_INFO(FYA_TAG"register fm_si4708 device successful! 1\n");
	fm_si4708_dev->client = client;
	
	return 0;
out:
	printk(KERN_ERR"%s:Driver Initialisation failed\n",__FILE__);
init_exit:

	kfree(fm_si4708_dev);

	return res;
}


static int si4708_remove(struct i2c_client *client)
{

	misc_deregister(&si4708_device);

	i2c_release_client(client);
	return 0;
}

static const struct i2c_device_id si4708_id[] = { 
	{ "si4708", 0 },
	{},
};

struct i2c_driver si4708_driver = {
	.probe  = si4708_probe,
	.remove = si4708_remove,
	.driver = {
		.name = "si4708",
		.owner = THIS_MODULE,
	},
	.id_table   = si4708_id,
};

int __init i2c_si4708_init(void)
{
	int res;

	res = i2c_add_driver(&si4708_driver);
	if (res) 
		FM_DBG(FYA_TAG"fm_si4708 add driver failed\n");
	FM_INFO(FYA_TAG"fm_si4708 add driver successful\n");
	
	return 0;
} 

void __exit  i2c_si4708_exit(void)
{

	i2c_del_driver(&si4708_driver);

}

MODULE_LICENSE("GPL");
MODULE_VERSION("2.0");
MODULE_AUTHOR("zte");
MODULE_DESCRIPTION("Silicon Laboratories'4708 semiconductor driver");
module_init(i2c_si4708_init);
module_exit(i2c_si4708_exit);
