#include <sys/select.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <time.h>
#include <assert.h>
//#include <libusb-1.0/libusb.h>

typedef struct { /* sizeof(trackinfo) == 64B */
	uint32_t unk0;
	char name[12];
	uint32_t timestamp;
	uint32_t duration;
	uint32_t length;
	uint32_t start_addr;
	uint32_t size;
	uint16_t unk1;
	uint16_t unk2;
	uint16_t unk3;
	uint16_t unk4;
	uint32_t unk5;
	uint32_t unk6;
	uint32_t unk7;
	uint32_t unk8;
	uint32_t unk9;
} trackinfo;

typedef struct { /* sizeof(waypoint) == 32B */
	uint32_t timestamp;
	float lat;
	float lon;
	uint16_t alt; //in m
	uint16_t speed; //in tenths of km/h

/* { 6, 4, "HEADING" },
   { 7, 2, "DSTA" },
   { 8, 4, "DAGE" },
   { 9, 2, "PDOP" },
   { 10, 2, "HDOP"},
   { 11, 2, "VDOP"},
   { 12, 2, "NSAT (USED/VIEW)"},
   { 13, 4, "SID",},
   { 14, 2, "ELEVATION" },
   { 15, 2, "AZIMUTH" },
   { 16, 2, "SNR"},
   { 17, 2, "RCR"},
   { 18, 2, "MILLISECOND"},
   { 19, 8, "DISTANCE" }, */
	uint16_t unk1;
	uint16_t unk2;
	uint16_t unk3;
	uint16_t unk4;
	uint16_t unk5;
	uint16_t unk6;
	uint32_t unk7;
} waypoint;

typedef enum {
	CMD_UNKNOWN = -3,
	CMD_QUIT = -2,
	CMD_NONE = -1,
	CMD_MODEL = 0,
	CMD_FWARE,
	CMD_START, //2
	CMD_TRACKCNT,
	CMD_TRACKS, //4 - better name
	CMD_1STSIZE,
	CMD_OFFSIZE, //6
	CMD_BLOCK,
	CMD_END = 8
} cmd_t;

struct tlist {
	trackinfo ti;
	unsigned num;
	struct tlist* prev;
};

#define ts_offset ((23*365+7*366)*24*3600)/*946684800*/ /* 1 Jan 2000 00:00 */

static const char* cmds[] = { //	answers:
	"PHLX810", //*35		$PHLX852,GR260*3E
	"PHLX829", //*3F		$PHLX861,201*2C - firmware version
	"PHLX826", //*30		$PHLX859*38 - disp. usb icon
	"PHLX701", //*3A		$PHLX601,18*1E - track count
	"PHLX702,0,", //*00		$PHLX900,702,3*33
		      //		$PHLX901,1152,FBFF1991*37 - 2nd line of response
	"PHLX900,901,3", //*3E		$PHLX902,0,1152,FBFF1991*28
	"PHLX900,902,3", //*3D		$PHLX902,30720,2048,AB690FC3*29
	"PHLX900,902,3",//*3D		binary data (264B+165+...)
//	"PHLX831",//*36			$PHLX863,GPSport260*74 #bye?
	"PHLX827",
/*9*/	"PHLX703,0,",//		
	"PHLX900,902,2", //*3C"		request retransmission of last packet
	NULL
};

static const char* rets[] = {
	"$PHLX852,GR260", //*3E
	"$PHLX861,", //201*2C //firmware version
	"$PHLX859*38",
	"$PHLX601,", //18*1E //number of tracks
	"$PHLX900,702,", //3*33
	"$PHLX901,",
	"$PHLX902,",
	"$PHLX900,703,3*32",
	NULL
};

static int send_cmd(const int fh, const char cmd[]) {
	char buf[32];
	unsigned i;
	int rv;
	unsigned char xors = 0;
			
	for (i = 0;cmd[i];i++) {
		xors ^= cmd[i];
	}
	i = sprintf(buf,"$%s*%02hhX\r\n",cmd,xors);
	rv = write(fh,buf,i);
	fprintf(stderr,"$%s*%02hhX -> ",cmd,xors);
	return rv;
}

static int my_strcmp(const char s1[], const char s2[]) {
	return strncmp(s1,s2,strlen(s2));
}

static void set_speed(const int fh, struct termios* const pterm,const speed_t speed) {
	cfsetispeed(pterm,speed);
	cfsetospeed(pterm,speed);
	if (tcsetattr(fh,TCSANOW,pterm) < 0) {
		perror("tcsetattr");
	}
}

static void dumpGpxHeader(FILE* f) {
	fprintf(f,"<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
		"<gpx\n"
		"  version=\"1.0\"\n"
		"  creator=\"GPSBabel - http://www.gpsbabel.org\"\n"
		"  xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\"\n"
		"  xmlns=\"http://www.topografix.com/GPX/1/0\"\n"
		"  xsi:schemaLocation=\"http://www.topografix.com/GPX/1/0 "
			"http://www.topografix.com/GPX/1/0/gpx.xsd\">\n");
}
static void dumpTrackHeader(FILE* f,unsigned tracknum) {
	fprintf(f,"<trk>\n"
		"  <name>track-%d</name>\n"
		"<trkseg>\n",tracknum);
	//<time>2011-06-28T20:27:31Z</time>
	//<bounds minlat=\"52.094039377\" minlon=\"20.592039437\" maxlat=\"52.310363814\" maxlon=\"21.030777570\"/>
	//  <desc>Log every 2 sec, 0 m</desc>
	// 
}

static void dumpTrackEnd(FILE* f) {
	fprintf(f,"</trkseg>\n</trk>\n");
}

static void dumpWaypoints(FILE* f,const char rbuf[], const int len,
			  const int gpxmode, int* const gpxheader,
			  struct tlist* const tl) {
	static unsigned wpnum,tracknum;
	struct tlist* itl;
	int i;

	for (i = 0; i < len; i+= sizeof(waypoint)) {
		waypoint *wp = (waypoint*)(rbuf+i);
		time_t t = wp->timestamp + ts_offset;
		struct tm* ptm = gmtime(&t);
		char tbuf[64];

		if (!gpxmode) {
			strftime(tbuf,sizeof(tbuf),"%F_%T",ptm);
			printf("%2d: %s %8.6f %8.6f %3d %2d  %5d %5d %5d %5d %5d %5d %u\n",
			       i/sizeof(waypoint)+1, tbuf, wp->lat, wp->lon,
			       wp->alt, (wp->speed+5)/10, wp->unk1, wp->unk2,
			       wp->unk3, wp->unk4, wp->unk5, wp->unk6, wp->unk7);
		} else {
			if (!*gpxheader) {
				dumpGpxHeader(f);
				*gpxheader = 1;
				wpnum = 0;
				tracknum = 1;
				dumpTrackHeader(f,tracknum);
			}
			for (itl = tl; itl; itl = itl->prev) {
				if (itl->ti.start_addr+itl->ti.size <= wpnum) {
					break;
				}
			}
			if (itl && itl->num != tracknum) {
				dumpTrackEnd(f);
				tracknum++;
				dumpTrackHeader(f,tracknum);
			}
			strftime(tbuf,sizeof(tbuf),"%FT%TZ",ptm);
			fprintf(f,"<trkpt lat=\"%.7f\" lon=\"%.7f\">\n"
				"  <ele>%d</ele>\n"
				"  <time>%s</time>\n"
				"  <speed>%d</speed>\n"
				"</trkpt>\n",wp->lat,wp->lon,wp->alt,tbuf,wp->speed);
			wpnum ++;
		}
	}
}

int main(const int argc, char* argv[]) {
	fd_set fds;
	FILE *gpxf = NULL;
	char rbuf[4*512+512]; //2KB is max anyway
	const char* tracksbin = NULL;
	struct termios nterm,oterm;
	struct timeval tv;
	struct tlist *tracklist = NULL;
	int i, fh = -1, fht = -1, rv, ridx = 0, nextcmd = CMD_MODEL, trackcnt=0, trackcurr = 1;
	int recvd = 0, hexmode = 0, hispeed = 0, gpxmode = 0, gpxheader = 0, listonly = 0;
	int opt, expbytes = -1, readtrack = -1;
//	libusb_device_handle *handle = NULL;
	void setQuit(int __attribute__((unused)) sno) {
		nextcmd = CMD_QUIT;
	}

	if (argc < 2) {
		goto printhelp;
	}
	close(0);
	while ((opt = getopt(argc,argv,"i:t:b:g:"/*c:d*/"lh")) != -1) {
		switch (opt) {
		case 'i':
			fh = open(optarg,O_RDWR|O_NOCTTY|O_NONBLOCK);
			if (fh < 0) {
				perror(optarg);
				return -1;
			}
			break;
		case 't':/* reads from 0 to given address */
			readtrack = strtol(optarg,NULL,0);
			break;
		case 'b':{
			char fn[1024] = "tracklist_";
			strncat(fn,optarg,sizeof(fn));
			fht = open(fn,O_WRONLY|O_CREAT,0644);
			if (fht < 0) {
				perror(fn);
			}
			tracksbin = optarg;
			};break;
		case 'g':
			gpxmode = 1;
			gpxf = fopen(optarg,"w");
			if (!gpxf) {
				perror(optarg);
			}
			break;
		case 'v':
			//verbose
			break;
		case 'd':
			//debug flag
			break;
		case 'c':
			//dump communication
			break;
		case 'l':
			listonly = 1;
			break;
		case 'h':
printhelp:
			printf("%s -i</dev/ttyUSB?> -t<track_offset> -b<memdump.bin> -g<file.gpx> -l -h\n",argv[0]);
			return 0;
		}
	}
	if (fh < 0) {
		abort();
	}
	signal(SIGINT,setQuit);
	tcgetattr(fh,&oterm);
	nterm = oterm;
	cfmakeraw(&nterm);
	set_speed(fh,&nterm,B38400);

	FD_ZERO(&fds);
	while (nextcmd != CMD_QUIT) {
		//static unsigned off = 0, readb = 0;//FIX
		if (nextcmd >= 0 && !recvd) {
			char cmdbuf[32];
			if (nextcmd == CMD_TRACKS) {
				sprintf(cmdbuf,"%s%d",cmds[nextcmd],trackcnt);
			} else if (nextcmd == 9) {
				sprintf(cmdbuf,"%s%d",cmds[nextcmd],readtrack);				
			} else {
				strcpy(cmdbuf,cmds[nextcmd]);
			}
			send_cmd(fh,cmdbuf);
			hexmode = 0;
			//nextcmd = -1;//recvd = 0;
		}
		FD_SET(fh,&fds);
		if (nextcmd == CMD_OFFSIZE || nextcmd == CMD_BLOCK) {
			/* shorter timeout */
			tv.tv_sec = 0;
			tv.tv_usec = 20*1000; //50
		} else {
			tv.tv_sec = 2;
			tv.tv_usec = 0;
		}
		rv = select(fh+1,&fds,NULL,NULL,&tv);
//		printf("select: rv:%d recvd:%d nextcmd:%d ptv:%p\n",rv,recvd,nextcmd,&tv);
		if (rv > 0 && FD_ISSET(fh,&fds)) {
//			printf("\nread: %d, hexmode:%d recvd:%d",ridx,hexmode,recvd);
			recvd = 1;
			i = read(fh,rbuf+ridx,sizeof(rbuf)-ridx);
			/*if (off >= 421888) {
				int k;
				fprintf(stderr,"%06u+%03u[%2u]:",off,ridx,i);
				for (k = 0; k < i; k++) {
					if ((rbuf+ridx)[k] > 32 && (rbuf+ridx)[k] < 127)
					fprintf(stderr,"  %c",(rbuf+ridx)[k]);
					else
					fprintf(stderr," %2hhX",(rbuf+ridx)[k]);
				}
				fprintf(stderr,"\n");
			}*/
			if (i < 0) {
				perror("read");
				//goto retry
			}
			assert(i >= 0);
			ridx += i;
			int j, ir = 0;
			for (ir = ridx-i; ir < ridx; ir++) {
				unsigned char c = rbuf[ir];
			if (c >= 32 && c < 127 && !hexmode) {
				fputc(c,stderr);
			} else if (c == '\r' && !hexmode) {
				fprintf(stderr,"\\r");
			} else if (c == '\n' && !hexmode) {
				for (j = ir; j >= 0; j--) {
					if (rbuf[j] == '$')
						break;
				}
				if (j < 0) {
					goto skip;
				}
				//- not necessary rbuf[ridx] = 0;
			//	if (j != 0 && j != 2048)
				//if (nextcmd != 6)
				//fprintf(stderr,"ridx:%d rbuf:%.*s hexmode:%d i:%d ir:%d j:%d\n",
				//	ridx,(ir-j)-2,rbuf+j,hexmode,i,ir,j);
			//	fprintf(stderr,"ridx:%d rbuf:%c,%c i:%d ir:%d j:%d\n",
			//		ridx,rbuf[0],rbuf[1],i,ir,j);
				recvd = 0;
				if (!my_strcmp(rbuf,rets[CMD_MODEL])) {
					printf("GR260 found.\n");
					nextcmd = CMD_FWARE;
				} else if (!my_strcmp(rbuf,rets[CMD_FWARE])) {
					unsigned fwver = atoi(rbuf+strlen(rets[CMD_FWARE]));
					printf("Firmware version: %u.%02u\n",fwver/100,fwver%100);
					nextcmd = CMD_START;
				} else if (!my_strcmp(rbuf,rets[CMD_START])) {
					if (!hispeed) {
						set_speed(fh,&nterm,B921600);
						hispeed = 1;
					}
					if (readtrack >=0) {
						nextcmd = 9;
					} else {
						nextcmd = CMD_TRACKCNT;
					}
				} else if (!my_strcmp(rbuf,rets[CMD_TRACKCNT])) {
					sscanf(rbuf+strlen(rets[CMD_TRACKCNT]),"%d",&trackcnt);
					nextcmd = 4;
					//printf("trackcnt %d\n",trackcnt);
				} else if (!my_strcmp(rbuf,rets[4])) {
					recvd = 1;
//					nextcmd = -1;
				} else if (!my_strcmp(rbuf,"$PHLX863,GPSport260")) {
					nextcmd = CMD_NONE;
				} else if (!my_strcmp(rbuf,rets[5])) {
					nextcmd = 5; //6
				//} else if (!my_strcmp(rbuf,rets[7])) {
				//	printf ("RETS7 %s\n",rets[7]);
				//	recvd = 1;
				} else if (!my_strcmp(rbuf+j,rets[6])) {
					/* TODO: verify CRC */
					int offset;
					/*int srv =*/ sscanf(rbuf+j+strlen(rets[6]),"%d,%d,%*X*%*d",&offset,&expbytes);
					//printf("expbytes:%d  srv:%d\n",expbytes,srv);
					nextcmd = 6;
					hexmode = 1; //test
				} else {
					//nextcmd = -1;
				}
				ridx = 0;
				fputc(c,stderr);
			} else {
			//	printf(" 0x%02hhX",c); fflush(stdout);
				tv.tv_sec = 2;
				tv.tv_usec = 0;
				hexmode = 1;
			}
skip:;
			} //for
		} else if (!rv) { /* timeout */
			static unsigned off = 0;
			//printf(" recvd:%d ridx:%d trackinfo:%d\n",recvd,ridx,sizeof(trackinfo));
			if (recvd) {
				fputc('\n',stderr);
				if (fht >= 0) {
					if (ridx != expbytes) {
				//+		fprintf(stderr,"FAILED! %d!=%d\n",ridx,expbytes);
						nextcmd = 10;
						recvd = 0;
					ridx = 0;
					} else {
				//+		fprintf(stderr,"write(%d,%p,%d), off:%u expb:%d\n",fht,rbuf,ridx,off,expbytes);
						off += write(fht,rbuf,ridx);
					}
				}
				if (readtrack < 0) {
					for (i = 0; i < ridx; i+= sizeof(trackinfo)) {
					trackinfo *ti = (trackinfo*)(rbuf+i);
					struct tlist* ntl;

					ntl = malloc(sizeof(struct tlist));
					ntl->prev = tracklist;
					ntl->ti = *ti;
					ntl->num = trackcurr++;
					tracklist = ntl;

					time_t t = ti->timestamp + ts_offset;
					struct tm* ptm = gmtime(&t);
					char tbuf[64];

					strftime(tbuf,sizeof(tbuf),"%T",ptm);
					fprintf(stderr,"%2d: %08X %10s %s %5ds %6dm %4X@%06X"
						" %05d %05d %05d %05d %08X %08X %03X %03X %u\n",
						trackcurr-1, ti->unk0,
						ti->name[0] != '\377'?ti->name:"(none)",
						tbuf, ti->duration, ti->length, ti->size,
						ti->start_addr, ti->unk1, ti->unk2, ti->unk3, ti->unk4,
						ti->unk5, ti->unk6, ti->unk7, ti->unk8, ti->unk9);
					}
				} else {
					dumpWaypoints(gpxf,rbuf,ridx,gpxmode,&gpxheader,tracklist);
				}
				if (nextcmd != 10)
					nextcmd = 6;
				//ridx = 0; //++
			} else if (nextcmd ==6) {
				if (readtrack <= 0 && tracklist && !listonly) {
					/* TODO: test: what will happend if we try read beyond the end of data? */
					readtrack = tracklist->ti.start_addr+tracklist->ti.size;
					nextcmd = 9;
					if (fht >= 0) {
						close(fht);
						fht = open(tracksbin,O_WRONLY|O_CREAT,0644);
						off = 0;
						if (fht < 0) {
							perror(tracksbin);
						}
					}

				} else {
					nextcmd = CMD_QUIT;
				}
			}
			recvd = 0;
//			set_speed(fh,&nterm,B38400);
			//nextcmd = 6;
		}
	}
	if (!hispeed) {
		set_speed(fh,&nterm,B921600);
	}
	send_cmd(fh,cmds[CMD_END]); //write(fh,"$PHLX831*36\r\n",13);
	tcsetattr(fh,TCSANOW,&oterm);
	close(fh);
	if (fht >= 0) {
		close(fht);
	}
	if (gpxmode && gpxheader) {
		dumpTrackEnd(gpxf);
		fprintf(gpxf,"</gpx>\n");
		fclose(gpxf);
	}
	fputs("\nbye!",stderr);
	return 0;
}

/*	//code for using usb directly
	libusb_init(NULL);	
	handle = libusb_open_device_with_vid_pid(NULL,0x67B,0x2303);
	if (handle) {
		unsigned char buf[7] = {0x68, 0x02, 0x00, 0x80, 0x00, 0x00, 0x08};
		int rv;

		libusb_detach_kernel_driver(handle,0);
		libusb_claim_interface(handle,0);
		rv = libusb_control_transfer(handle, 0x21, 0x20, 0, 0, buf, 7, 0);
		usleep(0);
		libusb_release_interface(handle,0);
		libusb_attach_kernel_driver(handle,0);
		libusb_close(handle);
		printf("found device; sent: %d IO_ERROR:%d\n",rv,LIBUSB_ERROR_IO);
		libusb_exit(NULL);
	}*/
