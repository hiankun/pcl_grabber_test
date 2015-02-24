// The code was copied and modifed from
// http://robotica.unileon.es/mediawiki/index.php/PCL/OpenNI_tutorial_1:_Installing_and_testing#Testing_.28OpenNI_viewer.29
//
// The main modification was adding the inclusion of ``openni2_grabber.h''
// and some code for ASUS Xtion.
//
// The code was a workaround for test purpose.
// Note that the initial viewpoint of the viewer was inverted,
// and you might have to roll down the middle button of the mouse to see the point cloud.
//
// Author:  thk
// Date:    2015/02/16

#define XTION 1
#define KINECT 0
#if XTION
#include <pcl/io/openni2_grabber.h>
#endif
#if KINECT
#include <pcl/io/openni_grabber.h>
#endif

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>

#include <iostream>

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudptr(new pcl::PointCloud<pcl::PointXYZRGBA>);   // a cloud that will store color info
pcl::PointCloud<pcl::PointXYZ>::Ptr fallbackCloud(new pcl::PointCloud<pcl::PointXYZ>);      // a fallback cloud with just depth data
boost::shared_ptr<pcl::visualization::CloudViewer> viewer;                   // point cloud viewer object
pcl::Grabber* openniGrabber;                                                 // OpenNI grabber that takes data from the device
unsigned int filesSaved = 0;                                            // for the number of the clouds saved to disk
bool saveCloud(false), noColor(false);                                  // program control

void printUsage(const char* programName) {
    std::cout << "Usage: " << programName << " [options]"
        << std::endl
        << std::endl
        << "Options:\n"
        << std::endl
        << "\t<none>     start capturing from an OpenNI device.\n"
        << "\t-v FILE    visualize the given .pcd file.\n"
        << "\t-h         shows this help.\n";
}

//--this function is called every time the device has new data
void grabberCallback(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud) {
    if (! viewer->wasStopped())
        viewer->showCloud(cloud);

    if (saveCloud) {
        std::stringstream stream;
        stream << "inputCloud" << filesSaved << ".pcd";
        std::string filename = stream.str();
        if (pcl::io::savePCDFile(filename, *cloud, true) == 0) {
            filesSaved++;
            std::cout << "Saved " << filename << "." << std::endl;
        }
        else PCL_ERROR("Problem of saving %s...\n", filename.c_str());

        saveCloud = false;
    }
}

//-- for detecting when SPACE is pressed
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing) {
    if (event.getKeySym() == "space" && event.keyDown())
        saveCloud = true;
}

//-- creates, initializes and returns a new viewer
boost::shared_ptr<pcl::visualization::CloudViewer> createViewer() {
    boost::shared_ptr<pcl::visualization::CloudViewer> v(new pcl::visualization::CloudViewer("OpenNI viewer"));
    v->registerKeyboardCallback(keyboardEventOccurred);

    return (v);
}

int main(int argc, char** argv) {
    if (pcl::console::find_argument(argc, argv, "-h") >= 0) {
        printUsage(argv[0]);
        return -1;
    }

    bool justVisualize(false);
    std::string filename;
    if (pcl::console::find_argument(argc, argv, "-v") >= 0) {
        if (argc != 3) {
            printUsage(argv[0]);
            return -1;
        }

        filename = argv[2];
        justVisualize = true;
    } else if (argc !=1) {
        printUsage(argv[0]);
        return -1;
    }

#if XTION
    //-- initialise the openniGrabber for Xtion
    boost::shared_ptr<pcl::io::openni2::OpenNI2DeviceManager> deviceManager = pcl::io::openni2::OpenNI2DeviceManager::getInstance ();

    std::string device_id ("");
    pcl::io::OpenNI2Grabber::Mode depth_mode = pcl::io::OpenNI2Grabber::OpenNI_Default_Mode;
    pcl::io::OpenNI2Grabber::Mode image_mode = pcl::io::OpenNI2Grabber::OpenNI_Default_Mode;
    pcl::io::OpenNI2Grabber openniGrabber(device_id, depth_mode, image_mode);
#endif

    //--first mode, openc and show a cloud from disk
    if (justVisualize) {
        try {
            pcl::io::loadPCDFile<pcl::PointXYZRGBA>(filename.c_str(), *cloudptr);
        } catch (pcl::PCLException e1) {
            try {// ...and if it fails, fall back to just depth
                pcl::io::loadPCDFile<pcl::PointXYZ>(filename.c_str(), *fallbackCloud);
            } catch (pcl::PCLException e2) {
                return -1;
            }
             noColor = true;
        }

        std::cout << "Loaded " << filename << "..." << std::endl;
        if (noColor)
            std::cout << "this cloud has no RGBA color info..." << std::endl;
        else
            std::cout << "this coloud has RGBA color info..." << std::endl;
    }

    //--second mode, start fetching and displaying frames from the device
    else {
#if KINECT
        openniGrabber = new pcl::OpenNIGrabber();
        if (openniGrabber == 0)
            return -1;
#endif
        boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
            boost::bind(&grabberCallback, _1);
#if XTION
        openniGrabber.registerCallback(f);
#endif
#if KINECT
        openniGrabber->registerCallback(f);
#endif
    }

    viewer = createViewer();

    if (justVisualize) {
        if (noColor)
            viewer->showCloud(fallbackCloud);
        else
            viewer->showCloud(cloudptr);
    } else {
#if XTION
        openniGrabber.start();
#endif
#if KINECT
        openniGrabber->start();
#endif
    }

    //--main loop
    while (!viewer->wasStopped())
        boost::this_thread::sleep(boost::posix_time::seconds(1));

    if (!justVisualize) {
#if XTION
        openniGrabber.stop();
#endif
#if KINECT
        openniGrabber->stop();
#endif
    }


    return 0;
}
