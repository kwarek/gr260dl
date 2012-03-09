#include <assert.h>
#include <sys/select.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdarg.h>
#include <signal.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <time.h>
//#include <libusb-1.0/libusb.h>

#define COMMPRINTF(...) if (gCommDump) fprintf(gCommDump,__VA_ARGS__)
#define MIN(a,b) ((a)<(b) ? (a) : (b))

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
	uint16_t altgps; /* altitude in m */
	uint16_t speed; /* in tenths of km/h */
/* { 7, 2, "DSTA" },
   { 8, 4, "DAGE" },
   { 9, 2, "PDOP" },
   { 10, 2, "HDOP"},
   { 11, 2, "VDOP"},
   { 12, 2, "NSAT (USED/VIEW)"},
   { 13, 4, "SID",},
   { 15, 2, "AZIMUTH" },
   { 16, 2, "SNR"},
   { 17, 2, "RCR"},
   { 18, 2, "MILLISECOND"}, */
	uint8_t unk1;
	uint8_t unk2 :4;
	uint8_t is_poi :4;
	uint16_t hbr;    /* heartbeat rate */
	uint16_t altbar; /* barimetric altitude in m */
	uint16_t heading; /* heding azimuth in degrees */
	uint32_t dist; /* distance travelled in m */
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
	CMD_END = 8,
	CMD_REQTDATA,
	CMD_RETRY
} cmd_t;

struct tlist {
	trackinfo ti;
	unsigned num;
	struct tlist* prev;
};

struct plist {
	waypoint poi;
	struct plist* prev;
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
/*9*/	"PHLX703,",//			request transmission of data between <start> and <end>
	"PHLX900,902,2", //*3C"		request retransmission of last packet
	NULL
};

static const char* rets[] = {
	"$PHLX852,GR260", //*3E
	"$PHLX861,", //201*2C //firmware version
	"$PHLX859*38",
	"$PHLX601,", //18*1E //number of tracks
	"$PHLX900,702,", //3*33
/*5*/	"$PHLX901,",
	"$PHLX902,",
	"$PHLX900,703,3*32",
	NULL
};

static int gVerbose = 1;
static FILE* gCommDump = NULL;

static int send_message(const int fh, const char cmd[]) {
	char buf[32];
	unsigned i;
	int rv;
	unsigned char xors = 0;
			
	for (i = 0;cmd[i];i++) {
		xors ^= cmd[i];
	}
	i = sprintf(buf,"$%s*%02hhX\r\n",cmd,xors);
	rv = write(fh,buf,i);
	COMMPRINTF("$%s*%02hhX -> ",cmd,xors);
	return rv;
}

static int send_cmd(const int fh, const cmd_t cmd, ...) {
	char cmdbuf[32];
	va_list vl;

	va_start(vl,cmd);
	if (cmd == CMD_TRACKS) {
		const int trackcnt = va_arg(vl,int);
		sprintf(cmdbuf,"%s%d",cmds[cmd],trackcnt);
	} else if (cmd == CMD_REQTDATA) {
		const int startaddr = va_arg(vl,int);
		const int endaddr = va_arg(vl,int);
		sprintf(cmdbuf,"%s%d,%d",cmds[cmd],startaddr,endaddr);
	} else {
		strcpy(cmdbuf,cmds[cmd]);
	}
	va_end(vl);
	return send_message(fh,cmdbuf);
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

static void dumpPOIs(FILE* f,struct plist* poilist) {
	char tbuf[64];
	struct plist* tmp;
	unsigned wpnum = 0;

	for (tmp = poilist; tmp; tmp = tmp->prev)
		wpnum++;
	while (poilist) {
		waypoint* poi = &(poilist->poi);
		time_t t = poi->timestamp + ts_offset;
		struct tm* ptm = gmtime(&t);

		strftime(tbuf,sizeof(tbuf),"%FT%TZ",ptm);
		fprintf(f,"<wpt lat=\"%.7f\" lon=\"%.7f\">\n"
			"  <ele>%d</ele>\n"
			"  <time>%s</time>\n"
			"  <name>WP%06d</name>\n"
			"</wpt>\n",
			poi->lat,poi->lon,poi->altgps,tbuf,wpnum);

		wpnum--;
		tmp = poilist->prev;
		free(poilist);
		poilist = tmp;
	}
}

static void trackListPrepend(const char rbuf[], const int ridx,
				    struct tlist** const tl) {
	static int trackcurr = 1;
	int i;

	for (i = 0; i < ridx; i+= sizeof(trackinfo)) {
		trackinfo *ti = (trackinfo*)(rbuf+i);
		struct tlist* ntl;

		/* prepend track list with new entry */
		ntl = malloc(sizeof(struct tlist));
		ntl->prev = *tl;
		ntl->ti = *ti; /* copy data */
		ntl->num = trackcurr++;
		*tl = ntl;
	}
}

static void dumpTracks(const struct tlist* from,
		       const struct tlist* const to) {
	char tbuf[64];

	if (!to) {
		return;
	}
	while (from != to) {
		const struct tlist* tl = to;
		while (tl->prev != from) {
		       tl = tl->prev;
		}
		const trackinfo *ti = &(tl->ti);
		time_t t = ti->timestamp + ts_offset;
		struct tm* ptm = gmtime(&t);

		strftime(tbuf,sizeof(tbuf),"%T",ptm);
		printf("%2d: %08X %10s %s %5ds %6dm %4X@%06X"
			" %05d %05d %05d %05d %08X %08X %03X %03X %u\n",
			tl->num, ti->unk0,
			ti->name[0] != '\377'?ti->name:"(none)",
			tbuf, ti->duration, ti->length, ti->size,
			ti->start_addr, ti->unk1, ti->unk2, ti->unk3, ti->unk4,
			ti->unk5, ti->unk6, ti->unk7, ti->unk8, ti->unk9);
		from = tl;
	}
}

static void dumpWaypoints(FILE* f,const char rbuf[], const int len,
			  const int gpxmode, int* const gpxheader,
			  struct tlist* const tl, struct plist** const pl,
			  const int usealtbar) {
	static unsigned wpnum,tracknum;
	struct tlist* itl;
	int i;

	for (i = 0; i < len; i+= sizeof(waypoint)) {
		waypoint *wp = (waypoint*)(rbuf+i);
		time_t t = wp->timestamp + ts_offset;
		struct tm* ptm = gmtime(&t);
		char tbuf[64];
					      
		if (wp->is_poi) {
			struct plist *newpoi = malloc(sizeof(struct plist));
			newpoi->prev = *pl;
			newpoi->poi = *wp;
			(*pl) = newpoi;
		}
		if (!gpxmode) {
			strftime(tbuf,sizeof(tbuf),"%F_%T",ptm);
			printf("%2d: %s %8.6f %8.6f %3d %2d %3u %2u %1d %5d %5d %5d %5d %u\n",
			       i/sizeof(waypoint)+1, tbuf, wp->lat, wp->lon,
			       wp->altgps, (wp->speed+5)/10, wp->unk1, wp->unk2, wp->is_poi, wp->hbr,
			       wp->altbar, wp->heading, wp->dist, wp->unk7);
		} else {
			if (!*gpxheader) {
				dumpGpxHeader(f);
				*gpxheader = 1;
				wpnum = 0;
				tracknum = 1;
				dumpTrackHeader(f,tracknum);
			}
			for (itl = tl; itl; itl = itl->prev) {
				if (itl->ti.start_addr <= wpnum) {
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
				"  <course>%d</course>\n"
				"  <speed>%.6f</speed>\n",
				wp->lat,wp->lon,usealtbar ? wp->altbar : wp->altgps,
				tbuf,wp->heading,wp->speed/36.);
			if (wp->hbr) {
				fprintf(f,"  <extensions>\n"
					"    <gpxtpx:TrackPointExtension>\n"
					"    <gpxtpx:hr>%d</gpxtpx:hr>\n"
					"    </gpxtpx:TrackPointExtension>\n"
					"  </extensions>\n",wp->hbr);
			}
			fprintf(f,"</trkpt>\n");
			wpnum ++;
		}
	}
}

int main(const int argc, char* argv[]) {
	fd_set fds;
	FILE *gpxf = NULL;
	char rbuf[4*512+512]; //2KB is max anyway
	struct termios nterm,oterm;
	struct timeval tv;
	struct tlist *tracklist = NULL;
	struct plist* poilist = NULL;
	cmd_t nextcmd = CMD_MODEL;
	int i, rv, hin = -1, hdump = -1, ridx = 0, trackcnt=0, gpxmode = 0, gpxheader = 0;
	int recvd = 0, hexmode = 0, usealtbar = 0;
	int opt, expbytes = -1, endaddr = -1, totalsize = 0, hispeed = 0, listonly = 0;
	unsigned totalchksum = 0;
	static unsigned off = 0;

	void setQuit(int __attribute__((unused)) sno) {
		nextcmd = CMD_QUIT;
	}

	if (argc < 2) {
		goto printhelp;
	}
	while ((opt = getopt(argc,argv,"i:f:t:b:g:c:dvqlah")) != -1) {
		switch (opt) {
		case 'i':
			hin = open(optarg,O_RDWR|O_NOCTTY|O_NONBLOCK);
			if (hin < 0) {
				perror(optarg);
				return -1;
			}
			break;
		case 'f':
			hin = open(optarg,O_RDWR|O_NOCTTY|O_NONBLOCK);
			if (hin < 0) {
				perror(optarg);
				return -1;
			}
			read(hin,&totalsize,4);
			read(hin,&totalchksum,4);
			break;
		case 't':/* reads from 0 to given address */
			endaddr = strtol(optarg,NULL,0);
			break;
		case 'b':{
			hdump = open(optarg,O_WRONLY|O_CREAT,0644);
			if (hdump < 0) {
				perror(optarg);
			}
			};break;
		case 'g':
			gpxmode = 1;
			gpxf = fopen(optarg,"w");
			if (!gpxf) {
				perror(optarg);
			}
			break;
		case 'q':
			//quiet
			break;
		case 'v':
			gVerbose = 2;
			break;
		case 'd':
			//debug flag
			break;
		case 'c':
			gCommDump = fopen(optarg,"w");
			if (!gCommDump) {
				perror(optarg);
			}
			break;
		case 'l':
			listonly = 1;
			gVerbose = 2; /* we probably want to see the list */
			break;
		case 'a':
			usealtbar = 1;
			break;
		case 'h':
printhelp:
			printf("%s\n"
			       "\t-i</dev/ttyUSB?> read from device\n"
			       "\t-f<memdump.bin>  read from file\n"
			       "\t-t<end_address>  retrieve part of log\n"
			       "\t-b<memdump.bin>  write to file\n"
			       "\t-g<file.gpx>     dump in gpx format\n"
			       "\t-c<comm_log.txt> dump communication\n"
			       "\t-v               be verbose (show tracklist)\n"
			       "\t-l               list tracks only\n"
			       "\t-a               use barimetric altitude\n"
			       "\t-h               show this help\n",argv[0]);
			return 0;
		}
	}
	close(0);
	if (hin < 0) {
		abort();
	}
	if (totalsize > 0) { /* read from file */
		while (totalsize > 0) {
			const struct tlist *from = tracklist;
			ridx = read(hin,rbuf,MIN(sizeof(rbuf),(unsigned)totalsize));
			if (ridx < 0) {
				perror("read");
				break;
			}
			trackListPrepend(rbuf,ridx,&tracklist);
			totalsize -= ridx;
			if (gVerbose > 1) {
				dumpTracks(from,tracklist);
			}
		}
		read(hin,&totalsize,4);
		read(hin,&totalchksum,4);
		while (totalsize > 0) {
			ridx = read(hin,rbuf,MIN(sizeof(rbuf),(unsigned)totalsize));
			if (ridx < 0) {
				perror("read");
				break;
			}
			dumpWaypoints(gpxf,rbuf,ridx,gpxmode,&gpxheader,tracklist,&poilist,usealtbar);
			totalsize -= ridx;
		}
		goto end;
	}
	signal(SIGINT,setQuit);
	tcgetattr(hin,&oterm);
	nterm = oterm;
	cfmakeraw(&nterm);
	set_speed(hin,&nterm,B38400);

	FD_ZERO(&fds);
	while (nextcmd != CMD_QUIT) {
		if (nextcmd >= 0 && !recvd) {
			if (nextcmd == CMD_TRACKS) {
				send_cmd(hin,CMD_TRACKS,trackcnt);
			} else if (nextcmd == CMD_REQTDATA) {
				send_cmd(hin,CMD_REQTDATA,0,endaddr);
			} else {
				send_cmd(hin,nextcmd);
			}
			hexmode = 0;
			//recvd = 0;
		}
		if (nextcmd == CMD_OFFSIZE || nextcmd == CMD_BLOCK) {
			/* shorter timeout */
			tv.tv_sec = 0;
			tv.tv_usec = 50*1000;
		} else {
			tv.tv_sec = 2;
			tv.tv_usec = 0;
		}
		FD_SET(hin,&fds);
		rv = select(hin+1,&fds,NULL,NULL,&tv);
		//printf("select:%d ridx:%d recvd:%d nextcmd:%d hexmode:%d\n",rv,ridx,recvd,nextcmd,hexmode);
		if (rv > 0 && FD_ISSET(hin,&fds)) {
			recvd = 1;
			i = read(hin,rbuf+ridx,sizeof(rbuf)-ridx);
			if (i < 0) {
				perror("read");
				//goto retry
				abort();
			}
			ridx += i;
			if (ridx == expbytes) { /* don't wait for timeout */
				goto dumpdata;
			}
			int j, ir = 0;
			for (ir = ridx-i; ir < ridx; ir++) {
				unsigned char c = rbuf[ir];
			if (c >= 32 && c < 127 && !hexmode) {
				COMMPRINTF("%c",c);
			} else if (c == '\r' && !hexmode) {
				COMMPRINTF("\\r");
			} else if (c == '\n' && !hexmode) {
				for (j = ir; j >= 0; j--) {
					if (rbuf[j] == '$')
						break;
				}
				if (j < 0) {
					goto skip;
				}
				recvd = 0;
				if (!my_strcmp(rbuf,rets[CMD_MODEL])) {
					fprintf(stderr,"GR260 found.\n");
					nextcmd = CMD_FWARE;
				} else if (!my_strcmp(rbuf,rets[CMD_FWARE])) {
					unsigned fwver = atoi(rbuf+strlen(rets[CMD_FWARE]));
					fprintf(stderr,"Firmware version: %u.%02u\n",fwver/100,fwver%100);
					nextcmd = CMD_START;
				} else if (!my_strcmp(rbuf,rets[CMD_START])) {
					if (!hispeed) {
						set_speed(hin,&nterm,B921600);
						hispeed = 1;
					}
					if (endaddr >=0) {
						nextcmd = CMD_REQTDATA;
					} else {
						nextcmd = CMD_TRACKCNT;
					}
				} else if (!my_strcmp(rbuf,rets[CMD_TRACKCNT])) {
					sscanf(rbuf+strlen(rets[CMD_TRACKCNT]),"%d",&trackcnt);
					nextcmd = CMD_TRACKS;
					//printf("trackcnt %d\n",trackcnt);
				} else if (!my_strcmp(rbuf,rets[4])) {
					recvd = 1;
					//nextcmd = -1;
				} else if (!my_strcmp(rbuf,"$PHLX863,GPSport260")) {
					nextcmd = CMD_NONE;
				} else if (!my_strcmp(rbuf,rets[5])) {
					nextcmd = CMD_1STSIZE; //6
					sscanf(rbuf+strlen(rets[5]),"%d,%X*%*d",&totalsize,&totalchksum);
					off = 0;
					if (hdump >= 0) {
						write(hdump,&totalsize,sizeof(totalsize));
						write(hdump,&totalchksum,sizeof(totalchksum));
					}
				//} else if (!my_strcmp(rbuf,rets[7])) {
				//	printf ("RETS7 %s\n",rets[7]);
				//	recvd = 1;
				} else if (!my_strcmp(rbuf+j,rets[6])) {
					/* TODO: verify CRC */
					int offset;
					/*int srv =*/ sscanf(rbuf+j+strlen(rets[6]),"%d,%d,%*X*%*d",&offset,&expbytes);
					//printf("expbytes:%d  srv:%d\n",expbytes,srv);
					nextcmd = CMD_OFFSIZE;
					hexmode = 1; //test
				} else {
					//nextcmd = -1;
				}
				ridx = 0;
				COMMPRINTF("%c",c);
			} else {
			//	printf(" 0x%02hhX",c); fflush(stdout);
				tv.tv_sec = 2;
				tv.tv_usec = 0;
				hexmode = 1;
			}
skip:;
			} //for
		} else if (!rv) { /* timeout */
			if (recvd) {
dumpdata:
				COMMPRINTF("\n");
				if (hdump >= 0) {
					if (ridx != expbytes) {
						COMMPRINTF("FAILED! %d!=%d\n",ridx,expbytes);
						nextcmd = CMD_RETRY;
						ridx = 0;
					} else {
						//COMMPRINTF("write(%d,%p,%d), off:%u expb:%d\n",hdump,rbuf,ridx,off,expbytes);
						off += write(hdump,rbuf,ridx);
						fprintf(stderr,"\r%7d/%d",off,totalsize);
					}
				}
				if (endaddr < 0) {
					const struct tlist *from = tracklist;
					trackListPrepend(rbuf,ridx,&tracklist);
					if (gVerbose > 1) {
						dumpTracks(from,tracklist);
					}
				} else {
					dumpWaypoints(gpxf,rbuf,ridx,gpxmode,&gpxheader,tracklist,&poilist,usealtbar);
				}
				if (nextcmd != CMD_RETRY)
					nextcmd = CMD_OFFSIZE;
				recvd = 0;
				//ridx = 0; //++
			} else if (nextcmd == CMD_OFFSIZE) {
				if (endaddr <= 0 && tracklist && !listonly) {
					/* TODO: test: what will happend if we try read beyond the end of data? */
					endaddr = tracklist->ti.start_addr+tracklist->ti.size;
					nextcmd = CMD_REQTDATA;

				} else {
					nextcmd = CMD_QUIT;
				}
			}
		}
	}
	if (!hispeed) {
		set_speed(hin,&nterm,B921600);
	}
	send_cmd(hin,CMD_END);
	tcsetattr(hin,TCSANOW,&oterm);
end:
	close(hin);
	if (hdump >= 0) {
		close(hdump);
	}
	if (gCommDump) {
		fclose(gCommDump);
	}
	if (gpxmode && gpxheader) {
		dumpTrackEnd(gpxf);
		dumpPOIs(gpxf,poilist);
		fprintf(gpxf,"</gpx>\n");
		fclose(gpxf);
	}
	fprintf(stderr,"\nbye!\n");
	return 0;
}

/*	//code for using usb directly
	libusb_device_handle *handle = NULL;
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
