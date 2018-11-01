#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/fcntl.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/uaccess.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#include <linux/timex.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/cdev.h>
#include <asm/io.h>
#include <plat/gpio-cfg.h>

#include "brlkey.h"

/* Character device */
static struct class *brlkey_class;
unsigned int brlkey_major = 0;

struct hims_brlkey *ghims_brlkey;
#define KEYDATA_BASE	ghims_brlkey->keydat_base
struct task_struct *g_th_brlkey;
struct message_key gmsg_key;

static DEFINE_MUTEX(ghmutex_key);
static DEFINE_MUTEX(ghHW_key);

static int64_t dwLongKey, lastKeyData = 0xFFFFFFFF;
long start_key_t, end_key_t;
bool stopped_rcv_data;

wait_queue_head_t wait_data_rcv;

// const char* request_name[23] = {
// 	"h532b_key_interrupt_1",
// 	"h532b_key_interrupt_2",
// 	"h532b_scroll_interrupt_1",
// 	"h532b_scroll_interrupt_2",
// 	"h532b_key_select_1",
// 	"h532b_key_select_2",
// 	"h532b_key_select_3",
// 	"h532b_key_data_1",
// 	"h532b_key_data_2",
// 	"h532b_key_data_3",
// 	"h532b_key_data_4",
// 	"h532b_key_data_5",
// 	"h532b_key_data_6",
// 	"h532b_key_data_7",
// 	"h532b_key_data_8",
// 	"h532b_key_data_9",
// 	"h532b_key_data_10",
// 	"h532b_key_data_11",
// 	"h532b_key_data_12",
// 	"h532b_key_data_13",
// 	"h532b_key_data_14",
// 	"h532b_key_data_15",
// 	"h532b_key_data_16",
// };

static inline long myClock(void)
{
	struct timeval tv;
	do_gettimeofday(&tv);
	return (tv.tv_sec * 1000 + tv.tv_usec / 1000);
}

int64_t GetScanCode(void)
{
	uint64_t keydata = 0, scancode = 0, curcode = 0;

	/*Data 1*/
	gpio_set_value(ghims_brlkey->gpio_keysel[0], 0);
	gpio_set_value(ghims_brlkey->gpio_keysel[1], 1);
	gpio_set_value(ghims_brlkey->gpio_keysel[2], 1);

	keydata = __raw_readl(KEYDATA_BASE + 0xA4);
	keydata |= __raw_readl(KEYDATA_BASE + 0xC4) << 8;
	curcode = keydata & (0xFFFF);

	/*Data 2*/
	gpio_set_value(ghims_brlkey->gpio_keysel[0], 1);
	gpio_set_value(ghims_brlkey->gpio_keysel[1], 0);

	keydata = __raw_readl(KEYDATA_BASE + 0xA4);
	keydata |= __raw_readl(KEYDATA_BASE + 0xC4) << 8;
	curcode |= (keydata & (0xFFFF)) << 16;

	if(curcode == 0xFFFFFFFF || lastKeyData == 0xFFFFFFFF) {
		scancode |= curcode;
		lastKeyData = scancode;
	}
	else
		scancode |= lastKeyData;

	/*Data 3*/
	gpio_set_value(ghims_brlkey->gpio_keysel[1], 1);
	gpio_set_value(ghims_brlkey->gpio_keysel[2], 0);

	keydata = __raw_readl(KEYDATA_BASE + 0xA4);
	keydata |= __raw_readl(KEYDATA_BASE + 0xC4) << 8;
	scancode |= (keydata & (0x7FFF)) << 32;

	return scancode;
}

bool KeyPressed(void)
{
	bool bret = false;
	
	int reg = gpio_get_value(ghims_brlkey->gpio_brlkey);

	DBG_PRINTK("%s : keypressed = %d\r\n", __func__, reg);

	if( reg == 0 ) {
		bret = true;
	}
	return bret;
}

//YGRYU 2015.12.15 to test OTG
#ifdef CONFIG_H532B_DWC3_TEST						
extern int dwc3_otg_change_mode(int imode);
int gotgstate = 1;//device mode
#endif

// HIMS_H532B : start
#define HIMS_H532B_CURSORKEY_BASE       0x8000000          // refer to "kr/co/himsintl/h532b/BrailleKey.java"
static uint32_t convertH532BCursorKey(uint32_t key) {
    int i;
    uint32_t ret = HIMS_H532B_CURSORKEY_BASE;

    for(i = 0; i < 32; i++) {
        if(key & 0x1) {
            ret += i;
            break;
        }
        key = key >> 1;
    }
    
    return ret;
}

static uint32_t convertH532BBrailleKey(uint32_t key) {
    return key << 8;
}
// HIMS_H532B : end

static int brlkey_mainthread(void *arg)
{
	// struct hims_brlkey *brlkey = (struct hims_brlkey *)arg;
	bool bFirstKey = true, bDetected=false, bNoBrlKey=false, KeyRelease=false;
	unsigned short usKeyState=0;
	long start_t, end_t;
	uint32_t dwKeyValue=0, dwScanCode=0, dwScanCnt=0;
	uint32_t dwDebounce=1, dwAfterDebounce=0;
	uint32_t dwCursorKey=0; 
	uint64_t ScanCode, KeyBuf = 0, KeyValue = 0;
	uint64_t ulKeyMask;

	DBG_PRINTK("* brlkey_mainthread ++ \r\n");		
	
	while(1) {
		ghims_brlkey->stopped_key = 0;	
		wait_event_interruptible(ghims_brlkey->wait_key,
			ghims_brlkey->stopped_key);
		
		DBG_PRINTK("*%s : key event occur. Start scan code \r\n", __func__);
		
		GetScanCode();
		
		start_t = myClock();

		bFirstKey = 1;
		while(1) {
			wait_event_timeout(ghims_brlkey->wait, ghims_brlkey->stopped,
				msecs_to_jiffies(1));
			DBG_PRINTK("* starat_key_t setting\r\n");
			start_key_t = myClock();
			if( bFirstKey )
			{
				DBG_PRINTK("* bFirstKey \r\n");
				dwScanCnt=0;
				bFirstKey = false; bDetected = false;
				// usKeyState = STATE_READ;
				usKeyState = STATE_DEBOUNCE;
				dwKeyValue = 0; dwScanCode=0; dwCursorKey=0;
				bNoBrlKey = false; KeyRelease=false;
				dwDebounce = 2;
				ScanCode = 0x0;
				KeyBuf = 0x0;
				KeyValue = 0x0;
			} 
			dwScanCnt++;
			
			switch( usKeyState )
			{
			case STATE_DEBOUNCE:
				DBG_PRINTK("* STATE_DEBOUNCE \r\n");
				if( dwDebounce-- <= 0 )
				{
					usKeyState = STATE_READ;
					dwScanCnt = 0;
					bDetected = false;
					dwKeyValue = 0; dwScanCode=0;
					ScanCode = 0;
				}

				ScanCode = GetScanCode();
				
				// if( bAudioOnly ) {
				// 	ulKeyMask = KEY_AUDIO_MASK;
				// } else {
				// 	ulKeyMask = KEY_BUFF_MASK;
				// }
				ulKeyMask = (uint64_t)KEY_BUFF_MASK;
				ScanCode &= ulKeyMask;
				KeyRelease = (ScanCode == ulKeyMask) ? true : false;

				if( KeyRelease )
				{
					DBG_PRINTK("[BRLKEY] STATE_DEBOUNCE key release. scancode ="
						" 0x%X 0x%X \r\n", (unsigned int)ScanCode,
						(unsigned int)(ScanCode>>32));
					goto Err_NoBrlKey;			
				}
				break;

			case STATE_READ:
				DBG_PRINTK("* STATE_READ \r\n");
				ScanCode = GetScanCode() & ulKeyMask;
				KeyRelease = (ScanCode == ulKeyMask) ? 1 : 0;
					
				// DBG_PRINTK("[BRLKEY] ScanCode: 0x%X 0x%X \r\n", 
				// (unsigned int)ScanCode, (unsigned int)(ScanCode>>32));

				// 현재 눌린 키가 있는지 검사
				if( KeyRelease )
				{
					// if upper board not exists, too many disply 
					// DBG_PRINTK("[BRLKEY] KeyRelease ScanCode: 0x%X 0x%X\r\n",
					// (unsigned int)ScanCode, (unsigned int)(ScanCode>>32));

					bDetected = true;
					usKeyState = STATE_AFTER_DEBOUNCE;
					dwAfterDebounce = 0;
					continue;
				} else if( KeyBuf != ScanCode ) {
					KeyBuf = (ScanCode & (int64_t)KEY_BUFF_MASK);
					
					KeyValue |= (~ScanCode & (int64_t)KEY_BUFF_MASK);
					DBG_PRINTK("KeyValue: 0x%X 0x%X \r\n",
						(unsigned int)KeyValue,
						(unsigned int)(KeyValue>>32));

					dwScanCode = (ScanCode>>32)&0x7FFF;
					dwKeyValue = (KeyValue>>32)&0x7FFF;
					// DBG_PRINTK("dwScanCode:0x%X !dwScanCode:0x%X dwKeyValue"
					// ":0x%X \r\n", dwScanCode, (~dwScanCode), dwKeyValue);

					if( ((!(~dwScanCode & 0x60EE))&&(dwKeyValue & 0x60EE)) ||
						/*space,bs,enter,f1,f2,f3,f4*/
						((!(~dwScanCode & 0x7EEE))&&(dwKeyValue & 0x7EEE)) )
						/*space,bs,enter*/
					{
							bNoBrlKey = true;
							usKeyState = STATE_AFTER_DEBOUNCE_S;
							dwAfterDebounce = 0;
							continue;
					}
				}
				break;
			case STATE_DETECTED:
				DBG_PRINTK("* STATE_DETECTED \r\n");
				bDetected = true;
				continue;
			case STATE_AFTER_DEBOUNCE:
				DBG_PRINTK("* STATE_AFTER_DEBOUNCE \r\n");
				if (KeyPressed())
				{
					usKeyState = STATE_READ;
					bDetected = false;
					break;
				}
				if( dwAfterDebounce++ > AFTER_DEBOUNCE_COUNT ) {
					bDetected = true;
					break;
				}						
				break;
			case STATE_AFTER_DEBOUNCE_S:
				DBG_PRINTK("* STATE_AFTER_DEBOUNCE_S \r\n");
				ScanCode = GetScanCode();		
				
				KeyBuf = (ScanCode & (int64_t)KEY_BUFF_MASK);
				
				KeyValue |= (~ScanCode & (int64_t)KEY_BUFF_MASK);
				DBG_PRINTK("KeyValue: 0x%X 0x%X \r\n", (unsigned int)KeyValue,
					(unsigned int)(KeyValue>>32));
				
				dwScanCode = (ScanCode>>32)&0x7FFF;
				dwKeyValue = (KeyValue>>32)&0x7FFF;
					
				if( ! (((!(~dwScanCode & 0x60EE))) ||
					/*space,bs,enter,f1,f2,f3,f4*/ // YGRYU 2008.08.12
					((!(~dwScanCode & 0x7EEE))) ) )
					/*space,bs,enter*/
				{
					usKeyState = STATE_READ;
					continue;
				}
				if( dwAfterDebounce++ > AFTER_DEBOUNCE_COUNT_S ) 
				{
					bNoBrlKey = true;
					// Send Key Msg 
					dwKeyValue &= (int32_t)KEY_BUFF_MASK_HALF;
					if( dwKeyValue )
					{
						//start_key_t = myClock();	
						printk("BRLKEY: Send Key_S Msg 0x%X\r\n",
							dwKeyValue);
						//Send key
						gmsg_key.msg_type = MSG_BRAILLE_KEY;
	                    // HIMS_H532B : start
						gmsg_key.key_value = convertH532BBrailleKey(dwKeyValue);   // dwKeyValue;
                        // HIMS_H532B : end
						gmsg_key.key_flag = 0;
						stopped_rcv_data = 1;
						lastKeyData = 0xFFFFFFFF;
						wake_up(&wait_data_rcv);
					}
					dwKeyValue = 0;
					KeyValue = 0;
					
					usKeyState = STATE_READ;
					continue;
				}				
													
				break;					
			} // end switch
			
			if( bDetected )
			{
				if( bNoBrlKey == false )
				{
					end_t = myClock() - start_t;
					if( (end_t/1000) >= 1 ) {
						dwLongKey = 1;
					} else {
						dwLongKey = 0;
					}
					// dwKeyUp = GetTickCount();
					// if( (abs(dwKeyUp-dwKeyDown)) >= 1000 )
					// {
					// 	dwLongKey = TRUE;
					// } else {
					// 	dwLongKey = FALSE;
					// }
					
					// DBG_PRINTK("[BRLKEY] ** ulKeyMask: 0x%X 0x%X \r\n",
					// 	(unsigned int)ulKeyMask, (unsigned int)(ulKeyMask>>32));
					// DBG_PRINTK("[BRLKEY] ** KeyValue: 0x%X 0x%X ->",
					// 	(unsigned int)KeyValue, (unsigned int)(KeyValue>>32));
					KeyValue &= ulKeyMask;
					// DBG_PRINTK("[BRLKEY] ** KeyValue: 0x%X 0x%X \r\n",
					// (unsigned int)KeyValue, (unsigned int)(KeyValue>>32));
					
					dwCursorKey = (KeyValue & 0xFFFFFFFF);
					dwKeyValue = ((KeyValue>>32) & 0x00007FFF);
					
					if( dwCursorKey ) {
						//start_key_t = myClock();	
						printk("[BRLKEY] Send CursorKeyValue: 0x%X \r\n",
							(unsigned int)dwCursorKey);
						//Send key
						gmsg_key.msg_type = MSG_CURSOR_KEY;
                        // HIMS_H532B : start
						gmsg_key.key_value = convertH532BCursorKey(dwCursorKey);   // dwCursorKey
						printk("[BRLKEY] Send CursorKeyValue: 0x%X \r\n",
							(unsigned int)gmsg_key.key_value);
                        // HIMS_H532B : end
						gmsg_key.key_flag= 0;
						stopped_rcv_data	=1;
						lastKeyData = 0xFFFFFFFF;
						wake_up(&wait_data_rcv);
					}
					else if( dwKeyValue ) {
						// int64_t dwAudioMode = 0;
						// if(!(dwKeyValue & 0x7FFF) && (dwKeyValue&0x7F0000)) {
						// 	dwAudioMode = (int64_t)GetAudioMode();
						// 	dwLongKey |= (dwAudioMode<<16);
						// 	DBG_PRINTK("[BRLKEY] Send Media Key Msg. Media Mode"
						// " 0x%X\r\n", dwLongKey);
						// }
						
						//start_key_t = myClock();							
						printk("[BRLKEYdepab] Send BrlKeyValue 0x%X LongKey:"
							" %lld\r\n", (unsigned int)dwKeyValue, dwLongKey);
						//YGRYU 2015.12.15 to test OTG
#ifdef CONFIG_H532B_DWC3_TEST						
						if( dwKeyValue == 0x8 ) {//dot 1
							if(gotgstate==1) {
								gotgstate = 0;//Host Mode
								dwc3_otg_change_mode(gotgstate);
							} else {
								printk("[BRLKEY] OTG is device mode already !! \r\n");
							}
						} else if( dwKeyValue == 0x20 ) {//dot 4
							if(gotgstate==0) {
								gotgstate = 1;//Device Mode
								dwc3_otg_change_mode(gotgstate);
							} else {
								printk("[BRLKEY] OTG is host mode already !! \r\n");
							}						
						}
#endif						
								 
						//Send key
						gmsg_key.msg_type = MSG_BRAILLE_KEY;
                        // HIMS_H532B : start
						gmsg_key.key_value = convertH532BBrailleKey(dwKeyValue);   // dwKeyValue;
                        // HIMS_H532B : end
						gmsg_key.key_flag= 0;
						stopped_rcv_data	=1;
						lastKeyData = 0xFFFFFFFF;
						wake_up(&wait_data_rcv);
					}
				} 
Err_NoBrlKey:			
				break;
			}		
					
		}
	
	}
	
	return 0;
}

static irqreturn_t scrollkey_irq0(int irq, void *dev_id)
{	
	int bscrKeyValue = 0x8000;
	static int keypressstat = 0;
	static int start_t = 0, end_t = 0;

	// Pressed
	if(gpio_get_value(ghims_brlkey->gpio_scroll_3) == 0) {
		if(start_t == 0) start_t = myClock();
		keypressstat = 1;
		return IRQ_HANDLED;
	}
	else if(keypressstat == 0) { // Not changed
		return IRQ_HANDLED;
	} else { // Released
		keypressstat = 0;
	}

	end_t = myClock() - start_t;
	if( (end_t/1000) >= 1 ) {
		dwLongKey = 1;
	} else {
		dwLongKey = 0;
	}
	start_t = 0;

	DBG_PRINTK("* scrollkey_irq0 : irq = %d++ \r\n", irq);
	printk("[BRLKEY]%s Send BrlKeyValue 0x%X LongKey: %lld \r\n", __func__,
		(unsigned int)bscrKeyValue, dwLongKey);
									 
	//Send key
	dwLongKey = 0;
	gmsg_key.msg_type	= MSG_BRAILLE_KEY;
// HIMS_H532B : start
	gmsg_key.key_value = convertH532BBrailleKey(bscrKeyValue);   // bscrKeyValue;
// HIMS_H532B : end
	gmsg_key.key_flag	= 0;
	stopped_rcv_data	= 1;							
	wake_up(&wait_data_rcv);

	DBG_PRINTK("* scrollkey_irq0 -- \r\n");
	return IRQ_HANDLED;
}

static irqreturn_t scrollkey_irq1(int irq, void *dev_id)
{
	int bscrKeyValue = 0x10000;
	static int keypressstat = 0;
	static int start_t = 0, end_t = 0;

	if(gpio_get_value(ghims_brlkey->gpio_scroll_4) == 0) {
		if(start_t == 0) start_t = myClock();
		keypressstat = 1;
		return IRQ_HANDLED;
	}
	else if(keypressstat == 0) {
		return IRQ_HANDLED;
	} else {
		keypressstat = 0;
	}

	end_t = myClock() - start_t;
	if( (end_t/1000) >= 1 ) {
		dwLongKey = 1;
	} else {
		dwLongKey = 0;
	}
	start_t = 0;
	
	DBG_PRINTK("* scrollkey_irq1 : irq = %d++ \r\n", irq); // irq = 544?
	printk("[BRLKEY]%s Send BrlKeyValue 0x%X LongKey: %lld \r\n", __func__,
		(unsigned int)bscrKeyValue, dwLongKey);

									 
	//Send key
	dwLongKey = 0;
	gmsg_key.msg_type	= MSG_BRAILLE_KEY;
// HIMS_H532B : start
	gmsg_key.key_value = convertH532BBrailleKey(bscrKeyValue);   // bscrKeyValue;
// HIMS_H532B : end
	gmsg_key.key_flag	= 0;
	stopped_rcv_data	= 1;							
	wake_up(&wait_data_rcv);

	DBG_PRINTK("* scrollkey_irq1 -- \r\n");
	return IRQ_HANDLED;
}

static irqreturn_t brlcur_irq(int irq, void *dev_id)
{
	struct hims_brlkey *brlkey = (struct hims_brlkey *)dev_id;

	DBG_PRINTK("* brlcur_irq ++ \r\n");

	brlkey->stopped_key = 1;
	wake_up(&brlkey->wait_key);

	DBG_PRINTK("* brlcur_irq -- \r\n");
	return IRQ_HANDLED;
}

static irqreturn_t brlkey_irq(int irq, void *dev_id)
{
	struct hims_brlkey *brlkey = (struct hims_brlkey *)dev_id;

	DBG_PRINTK("* brlkey_irq ++ \r\n");

	if(KeyPressed() == 0 ) { // NEED REINFORCE
		DBG_PRINTK("* brlkey_irq -- wrong interrupt \r\n");
		return IRQ_HANDLED;
	}

	brlkey->stopped_key = 1;
	wake_up(&brlkey->wait_key);

	DBG_PRINTK("* brlkey_irq -- \r\n");
	return IRQ_HANDLED;
}

int configure_gpioport(struct platform_device *pdev)
{
	int pinNum = 0;
	struct device_node *node;

	node = pdev->dev.of_node;
	if (!node) {
		return -ENODEV;
	}

	for(pinNum = 0; pinNum < 3; pinNum++)
		ghims_brlkey->gpio_keysel[pinNum] = of_get_named_gpio(node,
			"keysel_gpios", pinNum);

	// for(pinNum = 0; pinNum < 3; pinNum++)
	// if(gpio_request(ghims_brlkey->gpio_keysel[pinNum], request_name[4 + pinNum]))
	// 	printk(" >>> %s : key_sel request error\r\n", __func__);

	// for(pinNum = 0; pinNum < 16; pinNum++)
	// 	if(gpio_request(ghims_brlkey->gpio_brlkey, request_name[7 + pinNum]))
	// 		printk(" >>> %s : key_data request error\r\n", __func__);
	
	return 0;	
}

int configure_gpioint(struct platform_device *pdev)
{
	int error/*, reg*/;
	irq_handler_t handler;
	struct device_node *node;

	node = pdev->dev.of_node;
	if (!node) {
		return -ENODEV;
	}

	ghims_brlkey->gpio_brlcur = of_get_named_gpio(node,"irq_gpios", 0);
	ghims_brlkey->gpio_brlkey = of_get_named_gpio(node,"irq_gpios", 1);
	ghims_brlkey->gpio_scroll_3 = of_get_named_gpio(node,"irq_gpios", 2);
	ghims_brlkey->gpio_scroll_4 = of_get_named_gpio(node,"irq_gpios", 3);

	// if(gpio_request(ghims_brlkey->gpio_brlcur, request_name[0]))
	// 	printk(" >>> %s : cur_gpio setting error\r\n", __func__);
	// if(gpio_request(ghims_brlkey->gpio_brlkey, request_name[1]))
	// 	printk(" >>> %s : key_gpio setting error\r\n", __func__);
	// if(gpio_request(ghims_brlkey->gpio_scroll_3, request_name[2]))
	// 	printk(" >>> %s : scrl3_gpio setting error\r\n", __func__);
	// if(gpio_request(ghims_brlkey->gpio_scroll_4, request_name[3]))
	// 	printk(" >>> %s : scrl4_gpio setting error\r\n", __func__);
	
	ghims_brlkey->irq_brlcur = gpio_to_irq(ghims_brlkey->gpio_brlcur);
	ghims_brlkey->irq_brlkey = gpio_to_irq(ghims_brlkey->gpio_brlkey);
	ghims_brlkey->irq_scroll_3 = gpio_to_irq(ghims_brlkey->gpio_scroll_3);
	ghims_brlkey->irq_scroll_4 = gpio_to_irq(ghims_brlkey->gpio_scroll_4);

	// IRQ
	handler = &brlcur_irq;

	error = request_irq(ghims_brlkey->irq_brlcur, handler, IRQF_TRIGGER_FALLING,
			  "hims-brlcur", ghims_brlkey);
	if (error) {
		DBG_PRINTK("unable to request IRQ %d\n", ghims_brlkey->irq_brlcur);
		goto err_exit;
	}

	handler = &brlkey_irq;

	error = request_irq(ghims_brlkey->irq_brlkey, handler, IRQF_TRIGGER_FALLING,
			  "hims-brlkey", ghims_brlkey);
	if (error) {
		DBG_PRINTK("unable to request IRQ %d\n", ghims_brlkey->irq_brlkey);
		goto err_exit;
	}
	
	handler = &scrollkey_irq0;
	
	error = request_irq(ghims_brlkey->irq_scroll_3, handler,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "hims-brlscr1", 
				ghims_brlkey);
	if (error) {
		DBG_PRINTK("unable to request IRQ %d\n", ghims_brlkey->irq_scroll_3);
		goto err_exit;
	}	
	
	handler = &scrollkey_irq1;

	error = request_irq(ghims_brlkey->irq_scroll_4, handler,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "hims-brlscr2",
				ghims_brlkey);
	if (error) {
		DBG_PRINTK("unable to request IRQ %d\n", ghims_brlkey->irq_scroll_4);
		goto err_exit;
	}	
	
	DBG_PRINTK("+++++ brlkey: %s: success \r\n", __func__);
	return 0;
	
err_exit:
	DBG_PRINTK("+++++ brlkey: %s: fail \r\n", __func__);
	return error;	
}

static ssize_t
brlkey_read(struct file *file, char __user *buf, size_t count,loff_t *ppos)
{
	int ret;
	ssize_t retval;

	DBG_PRINTK("brlkey_read ++ \r\n");
	stopped_rcv_data = 0;
	wait_event_interruptible(wait_data_rcv, stopped_rcv_data);

	mutex_lock(&ghmutex_key);
	retval = sizeof(gmsg_key);
	ret = copy_to_user(buf, &gmsg_key, sizeof(gmsg_key));
	if(ret < 0)
		DBG_PRINTK("brlkey_read: copy_to_user fail \r\n");
	mutex_unlock(&ghmutex_key);
	
	DBG_PRINTK("brlkey_read: retval %lu \r\n", retval);
	return retval;
}

static int brlkey_open(struct inode *inode, struct file *file)
{
	DBG_PRINTK("* brlkey: brlkey_open \r\n");
	return 0;
}

/* platform device driver probe/remove */
static int h532b_key_probe(struct platform_device *pdev)
{
	int ret = 0;
	u32 kedata_addr;

	// init ghims structure
	ghims_brlkey = kzalloc(sizeof(*ghims_brlkey), GFP_KERNEL);
	if( !ghims_brlkey ) {
		ret = -ENOMEM;
		DBG_PRINTK("brlkey_init: kzalloc fail \r\n");
		goto err_free_mem;
	}

	// Get key data gpio address
	if (of_property_read_u32(pdev->dev.of_node, "keydata_addr", &kedata_addr)) {
		DBG_PRINTK("failed to get keydata address.");
		goto err_free_mem;
	}

	ghims_brlkey->keydat_base = ioremap(kedata_addr, 0x1000);
	if (!ghims_brlkey->keydat_base) {
		ret = -EBUSY;
		DBG_PRINTK("ioremap fail for gpio1 abse");
		goto err_free_mem;
	}	

	// init wait queue
	stopped_rcv_data = 0;
	init_waitqueue_head(&wait_data_rcv);
	init_waitqueue_head(&ghims_brlkey->wait);
	init_waitqueue_head(&ghims_brlkey->wait_key);
	ghims_brlkey->stopped = false;

	/* Start up our irq thread */
	g_th_brlkey = kthread_run(brlkey_mainthread, ghims_brlkey, "brlkey");
	if (IS_ERR(g_th_brlkey)) {

		ret = PTR_ERR(g_th_brlkey);
		goto err_free_mem;
	}

	configure_gpioport(pdev);
	ret = configure_gpioint(pdev);
	if(ret != 0) return ret;

	return 0;
err_free_mem:
	if( ghims_brlkey->keydat_base ) {
		iounmap(ghims_brlkey->keydat_base);
	}
	kfree(ghims_brlkey);

	return ret;
}

static int h532b_key_remove(struct platform_device *pdev)
{
	platform_set_drvdata(pdev, NULL);
	kfree(ghims_brlkey);

	return 0;
}

/* of device */
static const struct of_device_id brlkey_of_match[] = {
	{
		.compatible = "hims,h532b-key-gpio",
	},
	{},
};
MODULE_DEVICE_TABLE(of, brlkey_of_match);

/* cdev */
static const struct file_operations brlkey_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.read		= brlkey_read,
	.open		= brlkey_open,
};

/* platform device */
static struct platform_driver h532b_key_driver = {
	.probe = h532b_key_probe,
	.remove = h532b_key_remove,
	.driver = {
		.name = "h532b_key",
		.owner = THIS_MODULE,
		.pm = NULL,
		.of_match_table = brlkey_of_match,
	},
};
module_platform_driver(h532b_key_driver);

static int __init brlkey_init(void)
{
	struct device *dev=NULL;
	int ret=0;
	
	DBG_PRINTK("* brlkey: brlkey_init ++");

	/* init CHDEV */
	brlkey_major = register_chrdev(0, DEVICE_NAME, &brlkey_fops);
	if (brlkey_major<0){
		DBG_PRINTK("brlkey failed to register a major number");
		return brlkey_major;
	}
	DBG_PRINTK("brlkey : registered correctly with major number %d",
		brlkey_major);

	brlkey_class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(brlkey_class)) {
		DBG_PRINTK("Failed to create the device -- brlkey");
		unregister_chrdev(brlkey_major, DEVICE_NAME);
		ret = PTR_ERR(brlkey_class);
		goto err;
	}
	DBG_PRINTK("brlkey: device class created correctly");
	
	dev = device_create(brlkey_class, NULL, MKDEV(brlkey_major, 0), NULL,
		DEVICE_NAME);
	if( !dev ) {
		DBG_PRINTK("init: device_create fail");
		goto err;		
	}
	DBG_PRINTK("brlkey: device class created correctly");
	/* init CHDEV END*/

	DBG_PRINTK("brlkey: brlkey_init -- ret:%d", ret);

	return 0;

err:
	DBG_PRINTK("brlkey_init -- error:0x%X", ret);
	return ret;	
}

static void __exit brlkey_exit(void)
{
	device_destroy(brlkey_class, MKDEV(brlkey_major, 0));
  	class_unregister(brlkey_class);
	class_destroy(brlkey_class);
	unregister_chrdev(brlkey_major, DEVICE_NAME);
}

module_init(brlkey_init);
module_exit(brlkey_exit);


MODULE_AUTHOR("Paul Gortmaker");
MODULE_LICENSE("GPL");
MODULE_ALIAS_MISCDEV(BRLKEY_MINOR);
