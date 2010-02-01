
#include "NidasItem.h"

NidasItem::NidasItem(Project *project, int row, NidasItem *parent)
{
    nidasObject = (void*)project;
    nidasType = PROJECT;
    // Record the item's location within its parent.
    rowNumber = row;
    parentItem = parent;
}

NidasItem::NidasItem(Site *site, int row, NidasItem *parent)
{
    nidasObject = (void*)site;
    nidasType = SITE;
    // Record the item's location within its parent.
    rowNumber = row;
    parentItem = parent;
}

NidasItem::NidasItem(DSMConfig *dsm, int row, NidasItem *parent)
{
    nidasObject = (void*)dsm;
    nidasType = DSMCONFIG;
    // Record the item's location within its parent.
    rowNumber = row;
    parentItem = parent;
}

NidasItem::NidasItem(DSMSensor *sensor, int row, NidasItem *parent)
{
    nidasObject = (void*)sensor;
    nidasType = SENSOR;
    // Record the item's location within its parent.
    rowNumber = row;
    parentItem = parent;
}

NidasItem::NidasItem(SampleTag *sampleTag, int row, NidasItem *parent)
{
    nidasObject = (void*)sampleTag;
    nidasType = SAMPLE;
    // Record the item's location within its parent.
    rowNumber = row;
    parentItem = parent;
}

NidasItem::NidasItem(Variable *variable, int row, NidasItem *parent)
{
    nidasObject = (void*)variable;
    nidasType = VARIABLE;
    // Record the item's location within its parent.
    rowNumber = row;
    parentItem = parent;
}

NidasItem::~NidasItem()
{
    QHash<int,NidasItem*>::iterator it;
    for (it = childItems.begin(); it != childItems.end(); ++it)
        delete it.value();
    // do not delete nidasObject; leave it in the Nidas tree for ~Project()
}

NidasItem *NidasItem::parent()
{
    return parentItem;
}


/* maybe try:
 * Project * NidasItem::operator static_cast<Project*>()
 *    { return static_cast<Project*>this->nidasObject; }
 * so we can: Project *project = (Project*)this;
 */

NidasItem *NidasItem::child(int i)
{
    if (childItems.contains(i))
        return childItems[i];

    /*
     * when we don't have row/child i then build all of the cached childItems
     *  we expect (at least 1st time) for all children/rows to be requested in sequence
     *  so building/caching all is worth it
     *
     * based on QT4 examples/itemviews/simpledommodel/domitem.cpp
     *  domitem builds only the new item requested and adds it to childItems
     * XXX figure out the short-circuit return above esp re deleted rows/children
     *     and i>childItems.size()
     */

  int j;
  switch(this->nidasType){

  case PROJECT:
    {
    Project *project = reinterpret_cast<Project*>(this->nidasObject);
    SiteIterator it;
    for (j=0, it = project->getSiteIterator(); it.hasNext(); j++) {
        Site* site = it.next();
        NidasItem *childItem = new NidasItem(site, j, this);
        childItems[j] = childItem;
        }
    break;
    }

  case SITE:
    {
    Site *site = reinterpret_cast<Site*>(this->nidasObject);
    DSMConfigIterator it;
    for (j=0, it = site->getDSMConfigIterator(); it.hasNext(); j++) {

            // XXX *** XXX (also in configwindow.cc)
            // very bad casting of const to non-const to get a mutable pointer to our dsm
            // *** NEEDS TO BE FIXED either here or in nidas::core
            //
        DSMConfig * dsm = (DSMConfig*)(it.next());

        NidasItem *childItem = new NidasItem(dsm, j, this);
        childItems[j] = childItem;
        }
    break;
    }

  case DSMCONFIG:
    {
    DSMConfig *dsm = reinterpret_cast<DSMConfig*>(this->nidasObject);
    SensorIterator it;
    for (j=0, it = dsm->getSensorIterator(); it.hasNext(); j++) {
        DSMSensor* sensor = it.next();
        NidasItem *childItem = new NidasItem(sensor, j, this);
        childItems[j] = childItem;
        }
    break;
    }

  case SENSOR:
    {
    DSMSensor *sensor = reinterpret_cast<DSMSensor*>(this->nidasObject);
    SampleTagIterator it;
    for (j=0, it = sensor->getSampleTagIterator(); it.hasNext(); j++) {
        SampleTag* sample = (SampleTag*)it.next(); // XXX cast from const
        NidasItem *childItem = new NidasItem(sample, j, this);
        childItems[j] = childItem;
        }
    break;
    }

  case SAMPLE:
    {
    SampleTag *sampleTag = reinterpret_cast<SampleTag*>(this->nidasObject);
    VariableIterator it = sampleTag->getVariableIterator();
    for (j=0; it.hasNext(); j++) {
        Variable* var = (Variable*)it.next(); // XXX cast from const
        NidasItem *childItem = new NidasItem(var, j, this);
        childItems[j] = childItem;
        }
    break;
    }

  default:
    return 0;
  }

  return childItems[i];
}

int NidasItem::row() const
{
    return rowNumber;
}

int NidasItem::childCount()
{
if (int i=childItems.count()) return(i); // childItems has children, return how many
if (child(0)) // force a buildout of childItems
 return(childItems.count()); // and then return how many
return(0);
}

QString NidasItem::name()
{
  switch(this->nidasType){

  case PROJECT:
    {
    Project *project = reinterpret_cast<Project*>(this->nidasObject);
    return(QString::fromStdString(project->getName()));
    }

  case SITE:
    {
    Site *site = reinterpret_cast<Site*>(this->nidasObject);
    const Project *project = site->getProject();
    std::string siteTabLabel = project->getName();
    if (project->getSystemName() != site->getName())
        siteTabLabel += "/" + project->getSystemName();
    siteTabLabel += ": " + site->getName();
    return(QString::fromStdString(siteTabLabel));
    }

  case DSMCONFIG:
    {
    DSMConfig *dsm = reinterpret_cast<DSMConfig*>(this->nidasObject);
    return(QString::fromStdString(dsm->getLocation()));
    }

  case SENSOR:
    {
    DSMSensor *sensor = reinterpret_cast<DSMSensor*>(this->nidasObject);
    if (sensor->getCatalogName().length() > 0)
        return(QString::fromStdString(sensor->getCatalogName()+sensor->getSuffix()));
    else return(QString::fromStdString(sensor->getClassName()+sensor->getSuffix()));
    }

  case SAMPLE:
    {
    SampleTag *sampleTag = reinterpret_cast<SampleTag*>(this->nidasObject);
    return QString("Sample %1").arg(sampleTag->getSampleId());
    }

  case VARIABLE:
    {
    Variable *var = reinterpret_cast<Variable*>(this->nidasObject);
    return QString::fromStdString(var->getName());
    }

  default:
    return QString();
  }

return QString();
}

QString NidasItem::value()
{
return QString("value");
}

QString NidasItem::dataField(int column)
{
if (column == 0) return name();

if (this->nidasType == SENSOR) {
    DSMSensor *sensor = reinterpret_cast<DSMSensor*>(this->nidasObject);
    switch (column) {
      case 1:
        return QString::fromStdString(sensor->getDeviceName());
      case 2:
        return QString::fromStdString(getSerialNumberString(sensor));
      case 3:
        return QString("(%1,%2)").arg(sensor->getDSMId()).arg(sensor->getSensorId());
      /* default: fall thru */
    }
  }

return QString();
}

int NidasItem::childColumnCount() const
{
if (this->nidasType == DSMCONFIG) return(4);
return(1);
}

std::string NidasItem::getSerialNumberString(DSMSensor *sensor)
// maybe move this to a helper class
{
    const Parameter * parm = sensor->getParameter("SerialNumber");
    if (parm) 
        return parm->getStringValue(0);

    CalFile *cf = sensor->getCalFile();
    if (cf)
        return cf->getFile().substr(0,cf->getFile().find(".dat"));

return(std::string());
}



static const QString _Site_Label("Site");

QVariant NidasItem::childLabel(int column) const
{
  switch(this->nidasType){

  case PROJECT:
    return _Site_Label;

  case SITE:
    return QString("DSM");

  case DSMCONFIG:
    {
    switch (column) {
      case 0:
        return QString("Sensor");
      case 1:
        return QString("Device");
      case 2:
        return QString("S/N");
      case 3:
        return QString("ID");
      /* default: fall thru */
      }
    }

  case SENSOR:
    return QString("Sample");

  case SAMPLE:
    return QString("Variable");

  case VARIABLE:
    return QString("xxx");

  break;

  /* default: fall thru */
  } // end switch

return QString("Name");
}
