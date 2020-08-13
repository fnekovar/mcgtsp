
#ifndef __INSERTION_H__
#define __INSERTION_H__

#include <memory>

struct Insertion{
    double SolutionCost;
    int TargetsIndex;
    int Direction;
    int Agent;
    int InsertIndex;

    Insertion(double l, int t, int d,  int a, int i) : SolutionCost(l), TargetsIndex(t), Direction(d), Agent(a), InsertIndex(i) {}

    // not used atm
    bool operator()(Insertion const & a, Insertion const & b) const
    {
        return a.SolutionCost < b.SolutionCost;
    }
};

struct InsComp{

    bool operator()(const std::shared_ptr<Insertion> a, const std::shared_ptr<Insertion> b) const
    {
        return a->SolutionCost < b->SolutionCost;
    }
  
  bool operator()(const Insertion* a, const Insertion* b) const  
  { 
      return a->SolutionCost < b->SolutionCost;
  }

};

#endif
