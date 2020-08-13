/*
 * File name: tmkgreed-etsp.cc
 * Date:      2016/12/07 08:33
 * Author:    Jan Faigl
 */

#include <boost/program_options.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <experimental/filesystem>

#include <crl/config.h>
#include <crl/logging.h>
#include <crl/perf_timer.h>
#include <crl/boost_args_config.h>

#include <crl/gui/guifactory.h>
#include <crl/gui/win_adjust_size.h>
#include <crl/gui/canvas.h>

#include "grasp-constained.h"
#include "WGS84toCartesian.hpp"

using crl::logger;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

const std::string GSOA_VERSION = "0.4";

typedef crl::gui::CCanvasBase Canvas;

/// ----------------------------------------------------------------------------
/// Program options variables
/// ----------------------------------------------------------------------------
std::string guiType = "none";

crl::CConfig guiConfig;
crl::CConfig mkgreedConfig;
std::string canvasOutput = "";

/// ----------------------------------------------------------------------------
/// Global variable
/// ----------------------------------------------------------------------------
crl::gui::CGui *g = 0;
#define GUI(x)  if(gui) { x;}

/// ----------------------------------------------------------------------------
bool parseArgs(int argc, char *argv[]) 
{
   bool ret = true;
   std::string configFile;
   std::string guiConfigFile;
   std::string loggerCfg = "";

   po::options_description desc("General options");
   desc.add_options()
      ("help,h", "produce help message")
      ("config,c", po::value<std::string>(&configFile)->default_value(std::string(argv[0]) + ".cfg"),
       "configuration file")
      ("logger-config,l", po::value<std::string>(&loggerCfg)->default_value(loggerCfg),
       "logger configuration file")
      ("config-gui", po::value<std::string>(&guiConfigFile)->default_value(std::string(argv[0]) + "-gui.cfg"),
       "dedicated gui configuration file")
      ;
   try {
      po::options_description guiOptions("Gui options");
      crl::gui::CGuiFactory::getConfig(guiConfig);
      crl::gui::CWinAdjustSize::getConfig(guiConfig);
      guiConfig.add<double>("gui-add-x",
            "add the given value to the loaded goals x coord to determine the canvas size and transformation",
            0);
      guiConfig.add<double>("gui-add-y",
            "add the given value to the loaded goals y coord to determine the canvas size and transformation",
            0);
      boost_args_add_options(guiConfig, "", guiOptions);
      guiOptions.add_options()
         ("canvas-output", po::value<std::string>(&canvasOutput), "result canvas outputfile");

      po::options_description mkgreedOptions("GSOA options");
      boost_args_add_options(CMKGREED::getConfig(mkgreedConfig), "", mkgreedOptions);

      po::options_description cmdline_options;
      cmdline_options.add(desc).add(guiOptions).add(mkgreedOptions);

      po::variables_map vm;
      po::store(po::parse_command_line(argc, argv, cmdline_options), vm);
      po::notify(vm);

      std::ifstream ifs(configFile.c_str());
      store(parse_config_file(ifs, cmdline_options), vm);
      po::notify(vm);
      ifs.close();
      ifs.open(guiConfigFile.c_str());
      store(parse_config_file(ifs, cmdline_options), vm);
      po::notify(vm);
      ifs.close();

      if (vm.count("help")) {
         std::cerr << std::endl;
         std::cerr << "GSOA solver ver. " << GSOA_VERSION << std::endl;
         std::cerr << cmdline_options << std::endl;
         ret = false;
      }
      if (
            ret &&
            loggerCfg != "" &&
            fs::exists(fs::path(loggerCfg))
         ) {
         crl::initLogger("mkgreed", loggerCfg.c_str());
      } else {
         crl::initLogger("mkgreed");
      }
      const std::string problemFile = mkgreedConfig.get<std::string>("problem");
      if (!fs::exists(fs::path(problemFile))) {
         ERROR("Problem file '" + problemFile + "' does not exists");
         ret = false;
      }
   } catch (std::exception &e) {
      std::cerr << std::endl;
      std::cerr << "Error in parsing arguments: " << e.what() << std::endl;
      ret = false;
   }
   return ret;
}

/// - main ---------------------------------------------------------------------
int main(int argc, char *argv[]) 
{
   Canvas *canvas = 0;
   int ret = -1;
   if (parseArgs(argc, argv)) {
      INFO("Start Logging");
      try {

         CMKGREED mkgreed(mkgreedConfig);
          CoordsVector pts;
          { // load problem
              crl::CPerfTimer t("Load problem time real:");
              std::string fname = mkgreedConfig.get<std::string>("problem").c_str();
              double dist = mkgreedConfig.get<double>("distance");
              Coords pt;
              Coords prev;
              const std::array<double, 2> reference = {mkgreedConfig.get<double>("base_lat"), mkgreedConfig.get<double>("base_lon")};
              Coords zeroCoord(0, 0);
              std::string empty;


              for (const auto &entry : std::experimental::filesystem::directory_iterator(fname)) {
                  std::ifstream in(entry.path().string());
                  in >> empty;
                  //in.ignore(std::numeric_limits<std::streamsize>::max(), ' ');
                  in >> prev;
                  const std::array<double, 2> prevWgs = {prev.x, prev.y};
                  const std::array<double, 2> prevCart = wgs84::toCartesian(reference, prevWgs);
                  prev = Coords(prevCart[0], prevCart[1]);
                  in.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                  while (in >> empty) {
                      in >> pt;
                      const std::array<double, 2> ptWgs = {pt.x, pt.y};
                      const std::array<double, 2> ptCart = wgs84::toCartesian(reference, ptWgs);
                      pt = Coords(ptCart[0], ptCart[1]);
                      if (zeroCoord.distance(pt) <= dist && zeroCoord.distance(prev) <= dist) {
                          pts.push_back(prev);
                          pts.push_back(pt);
                      }

                      prev = pt;
                      in.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                  }
              }
          }
          crl::gui::CWinAdjustSize::adjust(pts, guiConfig);
          if ((g = crl::gui::CGuiFactory::createGui(guiConfig)) != 0) {
              INFO("Start gui " + guiConfig.get<std::string>("gui"));
              canvas = new Canvas(*g);
          }
         mkgreed.setCanvas(canvas);
         { 
            crl::CPerfTimer t("Total solve time: ");
            mkgreed.solve();
         }
         INFO("End Logging");
         if (canvas) {
            if (canvasOutput.size()) {
               canvas->save(canvasOutput);
            }
            if (!guiConfig.get<bool>("nowait")) {
               INFO("click to exit");
               canvas->click();
            }
            delete canvas;
            delete g;
         }
      } catch (crl::exception &e) {
         ERROR("Exception " << e.what() << "!");
      } catch (std::exception &e) {
         ERROR("Runtime error " << e.what() << "!");
      }
      ret = EXIT_SUCCESS;
   }
   crl::shutdownLogger();
   return ret;
}
