/*
 * File name: gsoa.h
 * Date:      2016/12/07 08:32
 * Author:    Jan Faigl
 */

#ifndef __MKGREED_H__
#define __MKGREED_H__

#include <crl/config.h>
#include <crl/alg/algorithm.h>
#include <crl/gui/shape.h>

#include "coords.h"
#include "target.h"

   class CMKGREED : public crl::CAlgorithm {
      typedef crl::CAlgorithm Base;
      typedef std::vector<int> IntVector;
      public:
      static crl::CConfig &getConfig(crl::CConfig &config);

      CMKGREED(crl::CConfig &config);
      ~CMKGREED();

      std::string getVersion(void);
      std::string getRevision(void);

      void solve(void);

      protected:
      void load(void);
      void initialize(void);
      void after_init(void); 

      void iterate(int iter);
      double refine(int step, double errorMax);

      void save(void);
      void release(void);

      void defineResultLog(void);
      void fillResultRecord(int trial);

      private:
      void drawPath(void);
      void drawRing(int step);
      void savePic(int step, bool detail = false, const std::string &dir_suffix = "");
      
      TargetSetPtrVectorVector getTSSolution(TargetSetPtrVectorVector& prevSol, int nodes);
      void getG1Solution(TargetSetPtrVectorVector& prevSol);
      void getG2Solution(TargetSetPtrVectorVector& prevSol);
      void getG3Solution(TargetSetPtrVectorVector& prevSol);
      double get_path_cost(const TargetSetPtrVector &pts);
      double get_solution_cost(const TargetSetPtrVectorVector &pts);
      std::vector <std::shared_ptr < TargetSet>> copy(std::vector<std::shared_ptr < TargetSet>> const &input);
      std::vector <std::vector<std::shared_ptr < TargetSet>>> copy(std::vector <std::vector<std::shared_ptr < TargetSet>>> const &input);

      private:
      const bool SAVE_RESULTS;
      const bool SAVE_SETTINGS;
      const bool SAVE_INFO;

      const bool DRAW_RING_ITER;
      const bool DRAW_RING_ENABLE;
      const bool SAVE_PIC;

      crl::gui::CShape shapePath;
      crl::gui::CShape shapePathNodes;
      crl::gui::CShape shapeTargets;

      std::string method;

      const double dist;
      const double maxCost;
      const double pylonCost;

      const Coords base_coords;

      int m;
      
      TargetSetPtrVector targets;
      TargetSetPtrVectorVector finalSolution;

      double g1Score;
      double g2Score;
      double g3Score;
      double R_T_iterator;


      const double p1; // best group solution (p for prize)
      const double p2; // best tabu solution
      const double p3; // best grasp solution
      const int R_T;
      int sizeRCL; // autotuned
      const int sizeNeighborhood;
      const int sizeTabuList;
      const int tabuIterations;
      const int tabuPeriod;
      TargetSetPtrVectorVector neighborhood;

   };



#endif

/* end of gsoa.h */
