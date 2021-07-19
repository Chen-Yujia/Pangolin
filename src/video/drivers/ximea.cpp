/* This file is part of the Pangolin Project.
 * http://github.com/stevenlovegrove/Pangolin
 *
 * Copyright (c) 2015 Steven Lovegrove
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */


#include <pangolin/factory/factory_registry.h>
#include <pangolin/video/drivers/ximea.h>
#include <pangolin/video/iostream_operators.h>

namespace pangolin
{

// unpack 10p160 into 10p or 12p192 into 12p since Pangoling only understands 10p,12p
void XimeaVideo::UnpackAndRepack(unsigned char* out, unsigned char* in, int h, int w, int bpp) {
    const int pitch = (w*bpp)/8;
    for(int r=0; r<h; ++r) {
        uint8_t* pout = (uint8_t*)(out + r*pitch);
        uint8_t* pin = in + r*pitch;
        const uint8_t* pin_end = in + (r+1)*pitch;
        if(bpp==12) {
            while(pin != pin_end) {
                uint8_t* pins = pin + 16;
                for(int cnt = 0; cnt < 8; cnt++) {
                    const uint8_t msbs = int8_t(*(pins+cnt));
                    uint32_t val = (uint16_t( (uint16_t(*(pin++))<<4) | (msbs & 0x0f) )) << 12;
                    val |= uint16_t( (uint16_t(*(pin++))<<4) | ((msbs & 0xf0)>>4) );
                    *(pout++) = uint8_t( val & 0x0000FF);
                    *(pout++) = uint8_t( (val & 0x00FF00) >> 8);
                    *(pout++) = uint8_t( (val & 0xFF0000) >> 16);
                }
                pin += 8;
            }
        } else {
            while(pin != pin_end) {
                uint8_t* pins = pin + 16;
                for(int cnt = 0; cnt < 4; cnt++) {
                    const uint8_t msbs = int8_t(*(pins+cnt));
                    uint64_t val = ( (uint64_t(*(pin++))<<2) | (msbs & 0x03) ) << 30;
                    val |= ( (uint64_t(*(pin++))<<2) | ((msbs & 0x0C)>>2) ) << 20;
                    val |= ( (uint64_t(*(pin++))<<2) | ((msbs & 0x30)>>4) ) << 10;
                    val |= ( (uint64_t(*(pin++))<<2) | ((msbs & 0xC0)>>6) );
                    *(pout++) = uint8_t( val & 0x00000000FF);
                    *(pout++) = uint8_t( (val & 0x000000FF00) >> 8);
                    *(pout++) = uint8_t( (val & 0x0000FF0000) >> 16);
                    *(pout++) = uint8_t( (val & 0x00FF000000) >> 24);
                    *(pout++) = uint8_t( (val & 0xFF00000000) >> 32);
                }
                pin += 4;
            }
        }
    }
}

XimeaVideo::XimeaVideo(const Params& p): sn(""), streaming(false), packed(false), trigger(0)
{
    XI_RETURN stat;
    memset(&x_image,0,sizeof(x_image));
    x_image.size = sizeof(XI_IMG);

    for(Params::ParamMap::const_iterator it = p.params.begin(); it != p.params.end(); it++) {
        if(it->first == "sn"){
            sn = it->second;
        }
    }

    if(sn=="") {
        stat = xiOpenDevice(0, &xiH);
        if (stat != XI_OK)
            throw pangolin::VideoException("XimeaVideo: Unable to open first Ximea camera.");
        char c_sn[100]="";
        xiGetParamString(xiH, XI_PRM_DEVICE_SN, c_sn, sizeof(sn));
        sn.assign(c_sn);
	    pango_print_info("XimeaVideo: camera sn:%s\n",c_sn);
    } else {
        stat = xiOpenDeviceBy(XI_OPEN_BY_SN, sn.c_str(), &xiH);
        if (stat != XI_OK)
            throw pangolin::VideoException("XimeaVideo: Unable to open Ximea camera with sn:" + sn);
    }

    for(Params::ParamMap::const_iterator it = p.params.begin(); it != p.params.end(); it++) {
        if(it->first == "bpp") {
            std::string val = it->second;
            const bool packed = (val[val.size()-1] == 'p');
            if(packed) {
                val = val.substr(0,val.size()-1);
            }
            const int bpp = std::stoi(val);
            if(bpp<=8){
                stat = xiSetParamInt(xiH, XI_PRM_IMAGE_DATA_FORMAT, XI_RAW8);
                stat += xiSetParamInt(xiH, XI_PRM_IMAGE_DATA_BIT_DEPTH, bpp);
                stat += xiSetParamInt(xiH, XI_PRM_OUTPUT_DATA_BIT_DEPTH, bpp);
            } else {
                if(packed) {
                    // see https://www.ximea.com/support/wiki/allprod/Transport_Data_Packing
                    stat = xiSetParamInt(xiH, XI_PRM_IMAGE_DATA_FORMAT, XI_FRM_TRANSPORT_DATA);
                    stat += xiSetParamInt(xiH, XI_PRM_OUTPUT_DATA_BIT_DEPTH, bpp);
                    stat += xiSetParamInt(xiH, XI_PRM_OUTPUT_DATA_PACKING, XI_ON);
                } else {
                    stat = xiSetParamInt(xiH, XI_PRM_IMAGE_DATA_FORMAT, XI_RAW16);
                    stat += xiSetParamInt(xiH, XI_PRM_OUTPUT_DATA_BIT_DEPTH, bpp);
                    stat += xiSetParamInt(xiH, XI_PRM_IMAGE_DATA_BIT_DEPTH, bpp);
                }
            }
            if(stat!= XI_OK) {
                pango_print_error("XimeaVideo: Unable to set pixel bit depth\n");
            }
        } else if(it->first == "size") {
            const ImageDim dim = p.Get<ImageDim>("size", ImageDim(0,0) );
            SetParameter("width"  , std::to_string(dim.x));
            SetParameter("height" , std::to_string(dim.y));
        } else if(it->first == "pos") {
            const ImageDim pos = p.Get<ImageDim>("pos", ImageDim(0,0) );
            SetParameter("offsetX"  , std::to_string(pos.x));
            SetParameter("offsetY" , std::to_string(pos.y));
        } else if(it->first == "roi") {
            const ImageRoi roi = p.Get<ImageRoi>("roi", ImageRoi(0,0,0,0) );
            SetParameter("width"  , std::to_string(roi.w));
            SetParameter("height" , std::to_string(roi.h));
            SetParameter("offsetX", std::to_string(roi.x));
            SetParameter("offsetY", std::to_string(roi.y));
        } else if(it->first == "sn"){
            // do nothing since cam is open already
        } else if(it->first == "trigger"){
            trigger = std::stoi(it->second);
            if (trigger != 0 && trigger != 3) {
                pango_print_error("XImeaVideo: Invalid trigger mode\n");
            }
            SetParameter("trigger_source", std::to_string(trigger));
        } else {
            SetParameter(it->first, it->second);
        }
    }

    // Read pixel format
    PixelFormat pfmt;
    int x_fmt = 0;
    stat = xiGetParamInt(xiH, XI_PRM_IMAGE_DATA_FORMAT, &x_fmt);
    if (stat != XI_OK)
        throw pangolin::VideoException("XimeaVideo: Error getting image format.");

    stat = xiGetParamInt(xiH, XI_PRM_OUTPUT_DATA_BIT_DEPTH, &bpp);
    if (stat != XI_OK)
        throw pangolin::VideoException("XimeaVideo: Error getting output bit depth.");

    int i_packed;
    stat = xiGetParamInt(xiH, XI_PRM_OUTPUT_DATA_PACKING, &i_packed);
    if (stat != XI_OK)
        throw pangolin::VideoException("XimeaVideo: Error getting output packing.");

    switch(x_fmt) {
        case XI_MONO8:
        case XI_RAW8:
            pfmt = pangolin::PixelFormatFromString("GRAY8");
            break;
        case XI_MONO16:
        case XI_RAW16:
            pfmt = pangolin::PixelFormatFromString("GRAY16LE");
            break;
        case XI_RGB24:
            pfmt = pangolin::PixelFormatFromString("RGB24");
            break;
        case XI_RGB32:
            pfmt = pangolin::PixelFormatFromString("RGB32");
            break;
        case XI_FRM_TRANSPORT_DATA:
            if(bpp == 10) {
                if(i_packed == XI_ON) {
                    pfmt = pangolin::PixelFormatFromString("GRAY10");
                    packed = true;
                    break;
                }
            } else if(bpp == 12) {
                if(i_packed == XI_ON) {
                    pfmt = pangolin::PixelFormatFromString("GRAY12");
                    packed = true;
                    break;
                }
            }
            [[fallthrough]];
        default:
            throw pangolin::VideoException("XimeaVideo: Unknown pixel format: " + std::to_string(x_fmt) + " bpp:" + std::to_string(bpp));
    }

    stat = xiGetParamInt(xiH, XI_PRM_HEIGHT, &h);
    if (stat != XI_OK)
        throw pangolin::VideoException("XimeaVideo: Error getting image height.");

    stat = xiGetParamInt(xiH, XI_PRM_WIDTH, &w);
    if (stat != XI_OK)
        throw pangolin::VideoException("XimeaVideo: Error getting image width.");

    // DEBUG ONLY: Understand the default options
    // int info = -1;
    // stat = xiGetParamInt(xiH, XI_PRM_TRG_SOURCE, &info);
    // if (stat != XI_OK)
    //     throw pangolin::VideoException("XimeaVideo: Error getting trigger source.");
    // else
    //     pango_print_info("XimeaVideo: trigger source:%d\n", info);

    // stat = xiGetParamInt(xiH, XI_PRM_TRG_SELECTOR, &info);
    // if (stat != XI_OK)
    //     throw pangolin::VideoException("XimeaVideo: Error getting trigger source.");
    // else
    //     pango_print_info("XimeaVideo: trigger selector:%d\n", info);

    // stat = xiGetParamInt(xiH, XI_PRM_COLOR_FILTER_ARRAY, &info);
    // if (stat != XI_OK)
    //     throw pangolin::VideoException("XimeaVideo: Error getting trigger source.");
    // else
    //     pango_print_info("XimeaVideo: color filter array:%d\n", info);
    // DEBUG ONLY ENDS

    const StreamInfo stream_info(pfmt, w, h, (w*pfmt.bpp) / 8, 0);
    streams.push_back(stream_info);
    size_bytes = w*h*pfmt.bpp/8;

    InitPangoDeviceProperties();
}

XimeaVideo::~XimeaVideo()
{
    XI_RETURN stat;
    stat = xiCloseDevice(xiH);
    if (stat != XI_OK)
        pango_print_error("XimeaVideo: Error closing device.");

}

std::string XimeaVideo::RenameCommonParams(const std::string& name) {
    std::string xname = name;
    // overrinde a few std strings so to make driver more similar to others
    // in Pangolin
    if (xname == "Gain") xname = "gain";
    if (xname == "ExposureTime") xname = "exposure";
    if (xname == "binning") xname = "downsampling";
    if (xname == "OffsetX") xname = "offsetX";
    if (xname == "OffsetY") xname = "offsetY";
    if (xname == "Width") xname = "width";
    if (xname == "Height") xname = "height";
    return xname;
}

bool XimeaVideo::GetParameter(const std::string& name, std::string& result)
{
    // list of pameter strings is in xApi.h
    XI_RETURN stat;
    int val;
    stat = xiGetParamInt(xiH, RenameCommonParams(name).c_str(), &val);
    if(stat == XI_OK) {
        result.assign(std::to_string(val));
        return true;
    } else {
        pango_print_error("XimeaVideo: GetParameter, param %s error\n", name.c_str());
        return false;
    }
}

bool XimeaVideo::SetParameter(const std::string& name, const std::string& value)
{
    // list of pameter strings is in xApi.h
    XI_RETURN stat;
    std::string n = RenameCommonParams(name);
    int v = stoi(value);
    stat = xiSetParamInt(xiH, n.c_str(), v);
    if(stat == XI_OK) {
        // keep track of exposure since we use it for timing
        if(n=="exposure") {
            exposure_us = v;
        }
        return true;
    } else {
        pango_print_error("XimeaVideo: SetParameter, param %s error\n", name.c_str());
        return false;
    }
}

void XimeaVideo::InitPangoDeviceProperties()
{
    XI_RETURN stat;
    // Store camera details in device properties
    device_properties["VendorName"] = "Ximea";
    device_properties["SerialNumber"] = sn;
    int id = 0;
    stat = xiGetParamInt(xiH, XI_PRM_DEVICE_MODEL_ID, &id);
    if(stat != XI_OK) {
        pango_print_error("XimeaVideo: error getting DeviceModelID\n");
    } else {
        device_properties["ModelID"] = std::to_string(id);
    }
    int sid = 0;
    stat = xiGetParamInt(xiH, XI_PRM_SENSOR_MODEL_ID, &sid);
    if(stat != XI_OK) {
        pango_print_error("XimeaVideo: error getting SensorModelID\n");
    } else {
        device_properties["SensorModelID"] = std::to_string(sid);
    }
    device_properties[PANGO_HAS_TIMING_DATA] = true;
}

//! Implement VideoInput::Start()
void XimeaVideo::Start()
{
    if (!streaming) {
        XI_RETURN stat = xiStartAcquisition(xiH);
        if (stat != XI_OK) {
            throw pangolin::VideoException("XimeaVideo: Error starting stream.");
        } else {
            streaming = true;
        }
    }
}

//! Implement VideoInput::Stop()
void XimeaVideo::Stop()
{
    if(streaming) {
        XI_RETURN stat = xiStopAcquisition(xiH);
        if (stat != XI_OK)
            throw pangolin::VideoException("XimeaVideo: Error stopping stream.");
        streaming = false;
    }
}

//! Implement VideoInput::SizeBytes()
size_t XimeaVideo::SizeBytes() const
{
    return size_bytes;
}

//! Implement VideoInput::Streams()
const std::vector<StreamInfo>& XimeaVideo::Streams() const
{
    return streams;
}

bool XimeaVideo::GrabNext(unsigned char* image, bool wait)
{
    if (trigger == 3) {
        SetParameter("trigger_software", std::to_string(1));
    }
    static basetime last = pangolin::TimeNow();
    // getting image from camera
    XI_RETURN stat;
    if(wait) {
        stat = xiGetImage(xiH, 5000, &x_image);
    } else {
        stat = xiGetImage(xiH, 0, &x_image);
    }
    basetime now = pangolin::TimeNow();
    if(stat == XI_OK) {
        if(packed) {
            UnpackAndRepack(image,(unsigned char*)x_image.bp, h, w, bpp);
        } else {
            memcpy(image,x_image.bp,x_image.bp_size);
        }
        frame_properties[PANGO_EXPOSURE_US] = picojson::value(exposure_us);
        uint64_t ct = uint64_t(x_image.tsUSec+x_image.tsSec*1e6);
        frame_properties[PANGO_CAPTURE_TIME_US] = picojson::value(ct);
        frame_properties[PANGO_ESTIMATED_CENTER_CAPTURE_TIME_US] = picojson::value(ct - exposure_us);
        frame_properties[PANGO_HOST_RECEPTION_TIME_US] = picojson::value(pangolin::Time_us(now));
        frame_properties["frame_number"] = picojson::value(x_image.nframe);

        last = now;
        if(x_image.padding_x!=0) {
            throw pangolin::VideoException("XimeaVideo: image has non zero padding, current code does not handle this!");
        }
        return true;
    } else {
        return false;
    }
}

//! Implement VideoInput::GrabNewest()
bool XimeaVideo::GrabNewest(unsigned char* image, bool wait)
{
    return GrabNext(image,wait);
}

//! Access JSON properties of device
const picojson::value& XimeaVideo::DeviceProperties() const
{
    return device_properties;
}

//! Access JSON properties of most recently captured frame
const picojson::value& XimeaVideo::FrameProperties() const
{
    return frame_properties;
}

PANGOLIN_REGISTER_FACTORY(XimeaVideo)
{
    struct XimeaVideoFactory final : public FactoryInterface<VideoInterface> {
        std::unique_ptr<VideoInterface> Open(const Uri& uri) override {
            return std::unique_ptr<VideoInterface>(new XimeaVideo(uri));
        }
    };

    auto factory = std::make_shared<XimeaVideoFactory>();
    FactoryRegistry<VideoInterface>::I().RegisterFactory(factory, 10, "ximea");
}

}
