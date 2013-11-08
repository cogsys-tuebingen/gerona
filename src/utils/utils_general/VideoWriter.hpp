#ifndef VIDEOWRITER_HPP
#define VIDEOWRITER_HPP

/// SYSTEM
#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>
#include <iomanip>

namespace bf3 = boost::filesystem3;

class ExternalVideoWriter {
public:
    ExternalVideoWriter()
        : open_(false), frame_(0)
    {}

    virtual ~ExternalVideoWriter()
    {
        save();
    }

    bool open(const std::string& filename, double fps, cv::Size frameSize, int bitrate = 9600) {
        filename_ = filename;
        fps_ = fps;
        bitrate_ = bitrate;

        std::stringstream ss;
        ss << filename << ".dir/";

        dir_ = ss.str();

        if(bf3::exists(dir_)) {
            bf3::remove_all(dir_);
        }
        open_ = bf3::create_directories(dir_);

        ss << "frame_";
        prefix_ = ss.str();
    }

    bool save() {
        std::stringstream command;
        command << "ffmpeg -qscale 5 -y -r " << fps_ << " -b " << bitrate_ << " -i " << dir_ << "frame_%06d.png " << filename_;

//        command << "convert -delay 10 -loop 1 " << dir_ << "/frame*.png " << filename_ << ".gif";
//        command << "&& cp " << dir_ << "/$(ls " << dir_ << " | grep .png | sort | head -n1) " << filename_ << ".first.png";
//        command << "&& cp " << dir_ << "/$(ls " << dir_ << " | grep .png | sort -r | head -n1) " << filename_ << ".last.png";

        int i = system(command.str().c_str());

        bf3::remove_all(dir_);
        return i == 0;
    }

    bool isOpened() const {
        return open_;
    }

    double fps() const {
        return fps_;
    }

    ExternalVideoWriter& operator << (const cv::Mat& image) {
        std::stringstream file;
        file << prefix_ << std::setfill('0') << std::setw(6) << frame_ << ".png";
        cv::imwrite(file.str(), image);

        ++frame_;
    }

private:
    bool open_;
    int frame_;
    double fps_;
    int bitrate_;
    bf3::path dir_;
    std::string filename_;
    std::string prefix_;
};

#endif // VIDEOWRITER_HPP
