#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <stdlib.h>
#include <getopt.h>
#include <pthread.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <linux/videodev2.h>

#define V4L_BUFFERS_DEFAULT	 6
#define V4L_BUFFERS_MAX		 16

int Dbg_Param = 0x1f;
#define TestAp_Printf(flag, msg...) ((Dbg_Param & flag)?printf(msg):flag)

#define TESTAP_DBG_USAGE	(1 << 0)
#define TESTAP_DBG_ERR		(1 << 1)
#define TESTAP_DBG_FLOW		(1 << 2)
#define TESTAP_DBG_FRAME	(1 << 3)
#define TESTAP_DBG_BW	    (1 << 4)

typedef void (*UVC_Notify)(void *data, int size, void *context);

UVC_Notify encode_notify_func = NULL;

static void UVC_RegisterNotify(UVC_Notify notify)
{
	encode_notify_func = notify;
}

static void UVC_UnRegisterNotify(void)
{
	encode_notify_func = NULL;
}

static void brvs_send_data(void *data, int size, unsigned int *id)
{
	if(encode_notify_func)
		(*encode_notify_func)(data, size, id);
}

static int video_open(const char *devname)
{
    struct v4l2_capability cap;
    int dev, ret;

    dev = open(devname, O_RDWR);
    if (dev < 0) {
        TestAp_Printf(TESTAP_DBG_ERR, "Error opening device %s: %d.\n", devname, errno);
        return dev;
    }

    memset(&cap, 0, sizeof(cap));
    ret = ioctl(dev, VIDIOC_QUERYCAP, &cap);
    if (ret < 0) {
        TestAp_Printf(TESTAP_DBG_ERR, "Error opening device %s: unable to query device.\n",
            devname);
        close(dev);
        return ret;
    }

    if ((cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) == 0) {
        TestAp_Printf(TESTAP_DBG_ERR, "Error opening device %s: video capture not supported.\n",
            devname);
        close(dev);
        return -EINVAL;
    }

    printf( "Device %s opened: %s.\n", devname, cap.card);
    return dev;
}

static int video_set_format(int dev, unsigned int w, unsigned int h, unsigned int format)
{
    struct v4l2_format fmt;
    int ret;

    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = w;
    fmt.fmt.pix.height = h;
    fmt.fmt.pix.pixelformat = format;
    fmt.fmt.pix.field = V4L2_FIELD_ANY;

    ret = ioctl(dev, VIDIOC_S_FMT, &fmt);
    if (ret < 0) {
        TestAp_Printf(TESTAP_DBG_ERR, "Unable to set format: %d.\n", errno);
        return ret;
    }

    TestAp_Printf(TESTAP_DBG_FLOW, "Video format set: width: %u height: %u buffer size: %u\n",
        fmt.fmt.pix.width, fmt.fmt.pix.height, fmt.fmt.pix.sizeimage);
    return 0;
}

static int video_set_framerate(int dev, int framerate, unsigned int *MaxPayloadTransferSize)
{
    struct v4l2_streamparm parm;
    int ret;

    memset(&parm, 0, sizeof(parm));
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    ret = ioctl(dev, VIDIOC_G_PARM, &parm);
    if (ret < 0) {
        TestAp_Printf(TESTAP_DBG_ERR, "Unable to get frame rate: %d.\n", errno);
        return ret;
    }

    TestAp_Printf(TESTAP_DBG_FLOW, "Current frame rate: %u/%u\n",
        parm.parm.capture.timeperframe.numerator,
        parm.parm.capture.timeperframe.denominator);

    parm.parm.capture.timeperframe.numerator = 1;
    parm.parm.capture.timeperframe.denominator = framerate;

    ret = ioctl(dev, VIDIOC_S_PARM, &parm);
    if (ret < 0) {
        TestAp_Printf(TESTAP_DBG_ERR, "Unable to set frame rate: %d.\n", errno);
        return ret;
    }

    if(MaxPayloadTransferSize)
        *MaxPayloadTransferSize = parm.parm.capture.reserved[0];

    ret = ioctl(dev, VIDIOC_G_PARM, &parm);
    if (ret < 0) {
        TestAp_Printf(TESTAP_DBG_ERR, "Unable to get frame rate: %d.\n", errno);
        return ret;
    }

    TestAp_Printf(TESTAP_DBG_FLOW, "Frame rate set: %u/%u\n",
        parm.parm.capture.timeperframe.numerator,
        parm.parm.capture.timeperframe.denominator);
    return 0;
}

static int video_reqbufs(int dev, int nbufs)
{
    struct v4l2_requestbuffers rb;
    int ret;

    memset(&rb, 0, sizeof(rb));
    rb.count = nbufs;
    rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    rb.memory = V4L2_MEMORY_MMAP;

    ret = ioctl(dev, VIDIOC_REQBUFS, &rb);
    if (ret < 0) {
        TestAp_Printf(TESTAP_DBG_ERR, "Unable to allocate buffers: %d.\n", errno);
        return ret;
    }

    TestAp_Printf(TESTAP_DBG_FLOW, "%u buffers allocated.\n", rb.count);
    return rb.count;
}

static int video_enable(int dev, int enable)
{
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    int ret;

    ret = ioctl(dev, enable ? VIDIOC_STREAMON : VIDIOC_STREAMOFF, &type);
    if (ret < 0) {
        TestAp_Printf(TESTAP_DBG_ERR, "Unable to %s capture: %d.\n",
            enable ? "start" : "stop", errno);
        return ret;
    }

    return 0;
}

static void *video_capture_pthread(void *context)
{
    int dev, ret;
    int do_record = 1;
	int framerate = 30;
    int pixelformat = V4L2_PIX_FMT_H264;

    double fps;

	unsigned int i;
	unsigned int delay = 0;
	unsigned int width = 1280;
	unsigned int height = 720;
    unsigned int nframes = (unsigned int)-1;
    unsigned int nbufs = V4L_BUFFERS_DEFAULT;

	void *mem0[V4L_BUFFERS_MAX];

    char *dev_name = "/dev/video1";

	struct v4l2_buffer buf0;
	struct timeval start, end, ts;

	/* Open the video device. */
    dev = video_open(dev_name);
    if (dev < 0)
        return NULL;

	/* Set the video format. */
    if (video_set_format(dev, width, height, pixelformat) < 0) {
        close(dev);
        return NULL;
    }

	/* Set the frame rate. */
    if (video_set_framerate(dev, framerate, NULL) < 0) {
        close(dev);
        return NULL;
    }

	/* Allocate buffers. */
    if ((int)(nbufs = video_reqbufs(dev, nbufs)) < 0) {
        close(dev);
        return NULL;
    }

	/* Map the buffers. */
	for (i = 0; i < nbufs; i++) {
		memset(&buf0, 0, sizeof(buf0));
		buf0.index = i;
		buf0.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf0.memory = V4L2_MEMORY_MMAP;
		ret = ioctl(dev, VIDIOC_QUERYBUF, &buf0);
		if (ret < 0) {
			TestAp_Printf(TESTAP_DBG_ERR, "Unable to query buffer %u (%d).\n", i, errno);
			close(dev);
			return NULL;
		}
		TestAp_Printf(TESTAP_DBG_FLOW, "length: %u offset: %10u     --  ", buf0.length, buf0.m.offset);

		mem0[i] = mmap(0, buf0.length, PROT_READ, MAP_SHARED, dev, buf0.m.offset);
		if (mem0[i] == MAP_FAILED) {
			TestAp_Printf(TESTAP_DBG_ERR, "Unable to map buffer %u (%d)\n", i, errno);
			close(dev);
			return NULL;
		}
		TestAp_Printf(TESTAP_DBG_FLOW, "Buffer %u mapped at address %p.\n", i, mem0[i]);
	}

	/* Queue the buffers. */
	for (i = 0; i < nbufs; i++) {
		memset(&buf0, 0, sizeof(buf0));
		buf0.index = i;
		buf0.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf0.memory = V4L2_MEMORY_MMAP;
		ret = ioctl(dev, VIDIOC_QBUF, &buf0);
		if (ret < 0) {
			TestAp_Printf(TESTAP_DBG_ERR, "Unable to queue buffer0(%d).\n", errno);
			close(dev);			
			return NULL;
		}
	}

	/* Start streaming. */
	video_enable(dev, 1);

	gettimeofday(&start, NULL);

	for (i = 0; i >= 0; i++) {
		/* Dequeue a buffer. */
		memset(&buf0, 0, sizeof buf0);
		buf0.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf0.memory = V4L2_MEMORY_MMAP;
		ret = ioctl(dev, VIDIOC_DQBUF, &buf0);
		if (ret < 0) {
			TestAp_Printf(TESTAP_DBG_ERR, "Unable to dequeue buffer0 (%d).\n", errno);
			close(dev);
			return NULL;
		}

	    gettimeofday(&ts, NULL);
		//TestAp_Printf(TESTAP_DBG_FRAME, "Frame[%4u] %u bytes %ld.%06ld %ld.%06ld\n ", i, buf0.bytesused, buf0.timestamp.tv_sec, buf0.timestamp.tv_usec, ts.tv_sec, ts.tv_usec);

		brvs_send_data(mem0[buf0.index], buf0.bytesused, &i);

		/* Requeue the buffer. */
		if (delay > 0)
			usleep(delay * 1000);

		ret = ioctl(dev, VIDIOC_QBUF, &buf0);
		if (ret < 0) {
			TestAp_Printf(TESTAP_DBG_ERR, "Unable to requeue buffer0 (%d).\n", errno);
			close(dev);
			return NULL;
		}

		fflush(stdout);
	}
}

static void uvc_send_data(void *data, int size, void *context)
{
	static FILE *rec_fp1 = NULL;
	char *rec_filename = "uvc_cam.h264";

	/* Record the H264 video file */
	if(rec_fp1 == NULL)
		rec_fp1 = fopen(rec_filename, "a+b");

    if(rec_fp1 != NULL)
        fwrite(data, size, 1, rec_fp1);

	TestAp_Printf(TESTAP_DBG_FRAME, "save frame_id %d data %d bytes.\n", *(unsigned int *)context, size);
	/*
		to do ...
	*/
}

int main(void)
{
	static pthread_t s_pid;

	UVC_RegisterNotify(uvc_send_data);
	pthread_create(&s_pid, NULL, video_capture_pthread, NULL);
	for(;;){
		usleep(10000);		
	}
}