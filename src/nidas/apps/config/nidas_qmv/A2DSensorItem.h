
#ifndef _A2DSENSOR_ITEM_H
#define _A2DSENSOR_ITEM_H

#include "SensorItem.h"
#include <nidas/dynld/raf/DSMAnalogSensor.h>
#include <nidas/core/SensorCatalog.h>
#include <nidas/core/CalFile.h>

using namespace nidas::core;
using namespace nidas::dynld::raf;

class A2DSensorItem : public SensorItem
{

public:
    A2DSensorItem(DSMAnalogSensor *sensor, int row, NidasModel *theModel, 
                  NidasItem *parent) ;
/*:
      SensorItem(sensor, row, theModel, parent) { 
            cerr << "A2DSensor Item, sensor * = " << sensor << "\n";};
*/

    NidasItem * child(int i);

 //   std::string devicename() { return this->dataField(1).toStdString(); }

    const QVariant & childLabel(int column) const { 
          if (column == 0) return NidasItem::_Variable_Label;
          if (column == 1) return NidasItem::_Channel_Label;
          if (column == 2) return NidasItem::_Sample_Label;
          if (column == 3) return NidasItem::_Rate_Label;
          if (column == 4) return NidasItem::_Volt_Label;
          if (column == 5) return NidasItem::_CalCoef_Label;
    }

    int childColumnCount() const {return 6;}

    QString getA2DTempSuffix();

    bool removeChild(NidasItem *item);

// at some point this should be protected.
//protected:  
        // get/convert to the underlying model pointers
    DSMAnalogSensor *getDSMAnalogSensor() const { return dynamic_cast<DSMAnalogSensor*>(_sensor); }

private:
    //DSMAnalogSensor * _sensor;

};

#endif