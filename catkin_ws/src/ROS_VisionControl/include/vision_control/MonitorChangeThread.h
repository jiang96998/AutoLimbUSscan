#ifndef MONITORCHANGETHREAD_H
#define MONITORCHANGETHREAD_H
#include <QMutex>
#include <QThread>
#include <QtDebug>
#include <iostream>
#include <Eigen/Dense>


class MonitorChangeThread : public QThread
{
  Q_OBJECT
signals:

void resultMove(bool);
public:
  MonitorChangeThread();
  int m_monitor_read_count;

//  std::vector<Eigen::VectorXd> m_monitorPointPoses;
//  int m_monitorPointsNum;

  //get distance(3D) between two optical markers
  double getMarkerDistance(Eigen::Vector3d point1, Eigen::Vector3d point2);

protected:
  virtual void run();

private:
  QMutex *m_mutex;
  //variables to read marker positions of last moment
  Eigen::Vector3d m_pre_start_marker, m_pre_mid1_marker, m_pre_mid2_marker, m_pre_mid3_marker, m_pre_end_marker;
  //variables to read current marker positions
  Eigen::Vector3d m_current_start_marker, m_current_mid1_marker, m_current_mid2_marker, m_current_mid3_marker, m_current_end_marker;
};

#endif // MONITORCHANGETHREAD_H
