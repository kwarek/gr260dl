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
//#include <libusb-1.0/libusb.h>

typedef struct {
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

typedef struct {
	uint32_t timestamp;
	float lat;
	float lon;
	uint16_t alt; //in m
	uint16_t speed; //in tenths of km/h

	uint16_t unk1;
	uint16_t unk2;
	uint16_t unk3;
	uint16_t unk4;
	uint16_t unk5;
	uint16_t unk6;
	uint32_t unk7;
} waypoint;

struct tlist {
	trackinfo ti;
	unsigned num;
	struct tlist* prev;
};

#define ts_offset 946684800

static const char* cmds[] = { //	answers:
	"PHLX810", //*35		$PHLX852,GR260*3E
	"PHLX829", //*3F		$PHLX861,201*2C
	"PHLX826", //*30		$PHLX859*38 - disp. usb icon
	//now important stuff
	"PHLX701", //*3A		$PHLX601,18*1E - track count
	"PHLX702,0,", //*00		$PHLX900,702,3*33
		      //		$PHLX901,1152,FBFF1991*37
	"PHLX900,901,3", //*3E		$PHLX902,0,1152,FBFF1991*28
	"PHLX900,902,3", //*3D		some binary data (264B+165+...)
	//ff + 62 + 243 + 31 + 298 + 75 + 244 + 28 + 170 = 1152
	"PHLX900,902,3",//*3D		nothing?
//	"PHLX831",//*36			$PHLX863,GPSport260*74 #bye?
	"PHLX827",
	"PHLX703,0,",//		
	NULL
};

static const char* rets[] = {
	"$PHLX852,GR260", //*3E
	"$PHLX861,201*2C",
	"$PHLX859*38",
	"$PHLX601,", //18*1E //number of tracks
	"$PHLX900,702,3*33",
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

static void dumpGpxHeader(void) {
	printf("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
		"<gpx\n"
		"  version=\"1.0\"\n"
		"  creator=\"GPSBabel - http://www.gpsbabel.org\"\n"
		"  xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\"\n"
		"  xmlns=\"http://www.topografix.com/GPX/1/0\"\n"
		"  xsi:schemaLocation=\"http://www.topografix.com/GPX/1/0 "
			"http://www.topografix.com/GPX/1/0/gpx.xsd\">\n");
}
static void dumpTrackHeader(unsigned tracknum) {
	printf("<trk>\n"
		"  <name>track-%d</name>\n"
		"<trkseg>\n",tracknum);
	//<time>2011-06-28T20:27:31Z</time>
	//<bounds minlat=\"52.094039377\" minlon=\"20.592039437\" maxlat=\"52.310363814\" maxlon=\"21.030777570\"/>
	//  <desc>Log every 2 sec, 0 m</desc>
	// 
}

static void dumpTrackEnd(void) {
		puts("</trkseg>\n</trk>");
}

static void dumpWaypoints(const char rbuf[], const int len, const int gpxmode,
			  int* const gpxheader, struct tlist* const tl) {
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
				dumpGpxHeader();
				*gpxheader = 1;
				wpnum = 0;
				tracknum = 1;
				dumpTrackHeader(tracknum);
			}
			for (itl = tl; itl; itl = itl->prev) {
				if (itl->ti.start_addr+itl->ti.size <= wpnum) {
					break;
				}
			}
			if (itl && itl->num != tracknum) {
				dumpTrackEnd();
				tracknum++;
				dumpTrackHeader(tracknum);
			}
			strftime(tbuf,sizeof(tbuf),"%FT%TZ",ptm);
			printf("<trkpt lat=\"%.7f\" lon=\"%.7f\">\n"
			       "  <ele>%d</ele>\n"
			       "  <time>%s</time>\n"
			       "  <speed>%d</speed>\n"
			       "</trkpt>\n",wp->lat,wp->lon,wp->alt,tbuf,wp->speed);
			wpnum ++;
		}
	}
}

int main(int argc, char* argv[]) {
	fd_set fds;
	char rbuf[4*512+64]; //2KB is max anyway
	struct termios nterm,oterm;
	struct timeval tv,*ptv = NULL;
	struct tlist *tracklist = NULL;
	int i,fh,fht = -1,rv,ridx = 0,nextcmd = 0, trackcnt=0, trackcurr = 1, readtrack = -1;
	int quit = 0, recvd = 0, hexmode = 0, hispeed = 0, gpxmode = 0, gpxheader = 0, listonly = 0;
//	libusb_device_handle *handle = NULL;
	void setQuit(int __attribute__((unused)) sno) {
		quit = 1;
	}

	if (argc < 2) {
		fprintf(stderr,"%s <device>\n",argv[0]);
		return -1;
	}
	close(0);
	signal(SIGINT,setQuit);
	FD_ZERO(&fds);
	fh = open(argv[1],O_RDWR|O_NOCTTY|O_NONBLOCK);
	if (fh < 0) {
		perror(argv[1]);
		return -1;
	}
	for (i = 1; i < argc; i++) {
		if (!strcmp(argv[i],"-t")) {
			readtrack = strtol(argv[i]+2,NULL,0);
		}
		if (!strcmp(argv[i],"-b")) {
			const char fn[] = "tracklist.bin";
			fht = open(fn,O_WRONLY|O_CREAT,0644);
			if (fht < 0) {
				perror(fn);
			}
		}
		if (!strcmp(argv[i],"-g")) {
			gpxmode = 1;
		}
		if (!strcmp(argv[i],"-l")) {
			listonly = 1;
		}
	}
	tcgetattr(fh,&oterm);
	nterm = oterm;
	cfmakeraw(&nterm);
	set_speed(fh,&nterm,B38400);

	while (!quit) {
		if (nextcmd >= 0 && !recvd) {
			char cmdbuf[32];
			if (nextcmd == 4) {
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
		ptv = &tv;
		FD_SET(fh,&fds);
		if (nextcmd == 6) {
			/* shorter timeout */
			tv.tv_sec = 0;
			tv.tv_usec = 20*1000;
		} else {
			tv.tv_sec = 2;
			tv.tv_usec = 0;
		}
		rv = select(fh+1,&fds,NULL,NULL,ptv);
//		printf("select: rv:%d recvd:%d nextcmd:%d ptv:%p\n",rv,recvd,nextcmd,ptv);
		if (rv > 0 && FD_ISSET(fh,&fds)) {
			unsigned char c;

			recvd = 1;
			//read(fh,&c,1);
			//rbuf[ridx++] = c;
			i = read(fh,rbuf+ridx,sizeof(rbuf)-ridx);
//			printf("\nread: %d, hexmode:%d ",i,hexmode);
			if (i > 0)
				ridx += i;
			int ir = 0;
			for (ir = 0; ir < i; ir++) {
				c = rbuf[ridx-i+ir];
			if (c > 32 && c < 127 && !hexmode) {
				fputc(c,stderr);
			} else if (c == '\r' && !hexmode) {
				fprintf(stderr,"\\r");
			} else if (c == '\n' && !hexmode) {
				rbuf[ridx] = 0;
				//printf("ridx:%d rbuf:%s hexmode:%d\n",ridx,rbuf,hexmode);
				if (!my_strcmp(rbuf,rets[0])) {
					recvd = 0;
					nextcmd = 1;
				} else if (!my_strcmp(rbuf,rets[1])) {
					recvd = 0;
					nextcmd = 2;
				} else if (!my_strcmp(rbuf,rets[2])) {

					recvd = 0;
					if (!hispeed) {
						//usleep(1000);
						set_speed(fh,&nterm,B921600);
						//usleep(10*1000);
						hispeed = 1;
					}
					if (readtrack >=0) {
						nextcmd = 9;
					} else {
						nextcmd = 3;
					}
				} else if (!my_strcmp(rbuf,rets[3])) {
					sscanf(rbuf+strlen(rets[3]),"%d",&trackcnt);
					recvd = 0;
					nextcmd = 4;
				} else if (!my_strcmp(rbuf,rets[7])) {
				} else if (!my_strcmp(rbuf,"$PHLX900,702,")) { // rets[4]
					//ridx = 0;
//					recvd = 0;
//					nextcmd = -1;
				} else if (!my_strcmp(rbuf,"$PHLX863,GPSport260")) {
					recvd = 0;
					nextcmd = -1;
				} else if (!my_strcmp(rbuf,rets[5])) {
					recvd = 0;
					nextcmd = 5; //6
//					tv.tv_sec = 2;
//					tv.tv_usec = 0;
//					ptv = &tv;
				} else if (!my_strcmp(rbuf,rets[6])) {
					/* TODO: verify CRC */
					recvd = 0;
					nextcmd = 6;
					hexmode = 1; //test
				} else {
					recvd = 0;
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
			}
		} else if (!rv) { /* timeout */
			//printf(" recvd:%d ridx:%d trackinfo:%d\n",recvd,ridx,sizeof(trackinfo));
			if (recvd) {
				fputc('\n',stderr);
				if (fht >= 0) {
					write(fht,rbuf,ridx);
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
					fprintf(stderr,"%2d: %08X %10s %s %5ds %5dm %4X@%06X"
						" %05d %05d %05d %05d %08X %08X %03X %03X %u\n",
						trackcurr-1, ti->unk0,
						ti->name[0] != '\377'?ti->name:"(none)",
						tbuf, ti->duration, ti->length, ti->size,
						ti->start_addr, ti->unk1, ti->unk2, ti->unk3, ti->unk4,
						ti->unk5, ti->unk6, ti->unk7, ti->unk8, ti->unk9);
					}
				} else {
					dumpWaypoints(rbuf,ridx,gpxmode,&gpxheader,tracklist);
				}
				nextcmd = 6;
			} else if (nextcmd ==6) {
				if (readtrack <= 0 && tracklist && !listonly) {
					/* TODO: test: what will happend if we try read beyond the end of data? */
					readtrack = tracklist->ti.start_addr+tracklist->ti.size;
					nextcmd = 9;
					if (fht >= 0) {
						const char fn[] = "tracks.bin";
						close(fht);
						fht = open(fn,O_WRONLY|O_CREAT,0644);
						if (fht < 0) {
							perror(fn);
						}
					}

				} else {
					quit = 1;
				}
			}
			recvd = 0;
//			set_speed(fh,&nterm,B38400);
			//nextcmd = 6;
			//ptv = NULL;
		}
	}
	if (!hispeed) {
		set_speed(fh,&nterm,B921600);
	}
	//write(fh,"$PHLX831*36\r\n",13);
	write(fh,"$PHLX827*31\r\n",13);
	tcsetattr(fh,TCSANOW,&oterm);
	close(fh);
	if (fht >= 0) {
		close(fht);
	}
	if (gpxmode && gpxheader) {
		dumpTrackEnd();
		puts("</gpx>");
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
