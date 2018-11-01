#ifndef H532B_BRLKEY
#define H532B_BRLKEY

#if 0
#define DBG_PRINTK(fmt, args...) printk("\t\tBRCELL %s: " fmt "\r\n", __func__, ##args)
#else
#define DBG_PRINTK(ARGS...)		do {} while(0);
#endif

#define DEVICE_NAME "brlcell"
#define CLASS_NAME	"cbrlcell"

#define BRAILLE_SIZE 	32 // Cell count

/* Defines for ioctl */
typedef struct
{
	unsigned char size;
	unsigned char data[BRAILLE_SIZE];
}__attribute__((packed))cell_data;

#define BRLCELL_SET_DATA				_IOW('C', 0x01, cell_data)

struct hims_brlcell {
	wait_queue_head_t wait;
	bool stopped;

	int gpio_clk;
	int gpio_data;
	int gpio_strb;
	int gpio_pwren_bra;
	int gpio_pwren_sys;
};

#endif
