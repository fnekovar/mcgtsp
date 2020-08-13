
#ifndef __TARSH_H__
#define __TARSH_H__

struct TargetShift{
    double costChange;
    int Agent1Index;
    int SourceIndex;
    int Agent2Index;
    int InsertIndex;
    bool reverse;

    TargetShift(double l, int a1, int s, int a2, int i, bool r) : costChange(l), Agent1Index(a1), SourceIndex(s), Agent2Index(a2), InsertIndex(i), reverse(r) {}

    // not used atm
    bool operator()(TargetShift const & a, TargetShift const & b) const
    {
        return a.costChange < b.costChange;
    }
};

struct TargetShiftComp{
  
  bool operator()(const TargetShift* a, const TargetShift* b) const  
  { 
      return a->costChange < b->costChange;
  }

  bool operator()(TargetShift a, TargetShift b) const
  {
      return a.costChange < b.costChange;
  }
};

#endif
