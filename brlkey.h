#ifndef H532B_BRLKEY
#define H532B_BRLKEY

#if 0
#define DBG_PRINTK(fmt, args...) printk("\t\tBRLKEY %s: " fmt "\r\n", __func__, ##args)
#else
#define DBG_PRINTK(ARGS...)		do {} while(0);
#endif

#define DEVICE_NAME	"brlkey"
#define CLASS_NAME	"cbrlkey"

#define MSG_BRAILLE_KEY	0x01
#define MSG_CURSOR_KEY	0x02

#define STATE_DEBOUNCE				1
#define STATE_READ 					2
#define STATE_POST_DEBOUNCE			3
#define STATE_DETECTED				4
#define STATE_AFTER_DEBOUNCE		5
#define STATE_AFTER_DEBOUNCE_S		6

#define AFTER_DEBOUNCE_COUNT		1 //3
#define AFTER_DEBOUNCE_COUNT_S		2 //12

#define CURSOR_KEY_MASK		0xFFFF
#define BRL_KEY_MASK		0x7FFF
#define MEDIA_KEY_MASK		0x001F

#define KEY_BUFF_MASK		0x7FFFFFFFFFFF
#define KEY_BUFF_MASK_HALF	0x00007FFF
#define KEY_AUDIO_MASK		0x001F000000000000

struct hims_brlkey {
	void __iomem *keydat_base;

	wait_queue_head_t wait;
	wait_queue_head_t wait_key;

	int gpio_brlcur;
	int gpio_brlkey;
	int gpio_scroll_3;
	int gpio_scroll_4;
	int gpio_keysel[3];
	int irq_brlcur;
	int irq_brlkey;
	int irq_scroll_3;
	int irq_scroll_4;

	bool stopped;
	bool stopped_key;
};

struct message_key {
	unsigned int msg_type;
	unsigned int key_value;
	unsigned int key_flag;
};

#endif
