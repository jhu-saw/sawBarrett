#include <cisstOSAbstraction/osaSleep.h>
#include <cisstOSAbstraction/osaGetTime.h>
#include <cisstRobot/robManipulator.h>
#include <cisstCommon/cmnPath.h>

#include <sawBarrett/osaWAM.h>
#include <sawCANBus/osaRTSocketCAN.h>
#include <native/task.h>
#include <sys/mman.h>

int main( int argc, char** argv ){

  mlockall(MCL_CURRENT | MCL_FUTURE);
  RT_TASK task;
  rt_task_shadow( &task, "GroupTest", 99, 0 );

  cmnLogger::SetMask( CMN_LOG_ALLOW_ALL );
  cmnLogger::SetMaskFunction( CMN_LOG_ALLOW_ALL );
  cmnLogger::SetMaskDefaultLog( CMN_LOG_ALLOW_ALL );

  if( argc != 2 ){
    std::cout << "Usage: " << argv[0] << " rtcan[0-1]" << std::endl;
    return -1;
  }

  osaRTSocketCAN can( argv[1], osaCANBus::RATE_1000 );

  if( can.Open() != osaCANBus::ESUCCESS ){
    CMN_LOG_RUN_ERROR << argv[0] << "Failed to open " << argv[1] << std::endl;
    return -1;
  }

  osaWAM WAM( &can );

  if( WAM.Initialize() != osaWAM::ESUCCESS ){
    CMN_LOG_RUN_ERROR << "Failed to initialize WAM" << std::endl;
    return -1;
  }

  vctDynamicVector<double> qinit( 7, 0.0 );
  qinit[1] = -cmnPI_2;
  qinit[3] =  cmnPI;
  
  if( WAM.SetPositions( qinit ) != osaWAM::ESUCCESS ){
    CMN_LOG_RUN_ERROR << "Failed to set position: " << qinit << std::endl;
    return -1;
  }


#if 1
  // Rotate the base
  vctMatrixRotation3<double> Rw0(  0.0,  0.0, -1.0,
                                   0.0,  1.0,  0.0,
                                   1.0,  0.0,  0.0 );
  vctFixedSizeVector<double,3> tw0(0.0);
  vctFrame4x4<double> Rtw0( Rw0, tw0 );

  cmnPath path;
//  path.AddRelativeToCisstShare("models/WAM");
//  path.AddRelativeToCisstRoot("share/models/WAM");
//  path.Add(cmnPath::GetCisstRoot() + "share/models/WAM");
  path.Add("/home/zihan/dev/cisst/source/share/models/WAM");

  robManipulator *manipulator = new robManipulator( path.Find("wam7.rob"), Rtw0 );

  std::cout << "robot file = " << path.Find("wam7.rob") << std::endl;

#endif



  double t1 = osaGetTime();
  size_t cnt=0;

  while( 1 ){

    vctDynamicVector<double> q;
    if( WAM.GetPositions( q ) != osaWAM::ESUCCESS ){
      CMN_LOG_RUN_ERROR << "Failed to get positions" << std::endl;
      return -1;
    }

    vctDynamicVector<double> tau( q.size(), 0.0 );
    if( WAM.SetTorques( tau ) != osaWAM::ESUCCESS ){
      CMN_LOG_RUN_ERROR << "Failed to set torques" << std::endl;
      return -1;
    }

#if 0
    std::cout << "q: " << q << std::endl;
    cnt++;
    if( cnt == 1000 ){
      double t2 = osaGetTime();
      std::cout << 1000.0 / (t2 - t1) << std::endl;
      t1 = t2;
      cnt = 0;
    }
#endif

#if 1
    vctFrame4x4<double> Rt;
    Rt = manipulator->ForwardKinematics( q );


//    std::cout << "q: " << q << std::endl;
    cnt++;
    if( cnt == 1000 ){
      double t2 = osaGetTime();
//      std::cout << 1000.0 / (t2 - t1) << std::endl;
      t1 = t2;
      cnt = 0;
      std::cout << "Rt: " << std::endl << Rt << std::endl;
    }
#endif


  }

  return 0;
}
