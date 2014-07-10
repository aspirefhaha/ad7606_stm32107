#ifndef _MPKG_H
#define _MPKG_H

#pragma pack(1)
#define PRECODE			0xAA
#define TAILCODE		0x55
struct _com_pkg {
	unsigned char precode;		//const 0xAA
	unsigned char op_type;		//operate type
	unsigned int op_value;	//operate result;
	unsigned char reserved;	//reserved
	unsigned char tailcode;	//const 0x55;
};

typedef struct _com_pkg com_pkg,*com_pkg_t;


#pragma pack()
int process_pkg(com_pkg_t pPkg);
#endif
