#include <cisstCommon/cmnGetChar.h>
#include <cisstCommon/cmnPath.h>
#include <cisstCommonXML.h>


#include <cisstMultiTask/mtsTaskManager.h>
#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>

#include <cisstOSAbstraction/osaSleep.h>
#include <cisstOSAbstraction/osaGetTime.h>

#include <cisstRobot/robManipulator.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <sawBarrett/mtsWAM.h>
#include <sawCANBus/osaRTSocketCAN.h>
#include <native/task.h>
#include <sys/mman.h>

class WAMprobe : public mtsTaskPeriodic {

private:
  
  mtsFunctionRead  GetPositions;
  mtsFunctionWrite SetTorques;

  prmPositionJointGet q;
  prmPositionCartesianGet cpos;
  robManipulator* manipulator;


public:

  WAMprobe() : mtsTaskPeriodic( "WAMprobe", 0.002, true ){

    // Initialize manipulator Rotate the base
    vctMatrixRotation3<double> Rw0(  0.0,  0.0, -1.0,
                                     0.0,  1.0,  0.0,
                                     1.0,  0.0,  0.0 );
    vctFixedSizeVector<double,3> tw0(0.0);
    vctFrame4x4<double> Rtw0( Rw0, tw0 );
    cmnPath path;
    path.Add("/home/zihan/dev/cisst/source/share/models/WAM");
    std::cout << "robot file = " << path.Find("wam7.rob") << std::endl;
    manipulator = new robManipulator( path.Find("wam7.rob"), Rtw0 );

    // Interface
    mtsInterfaceRequired* input = AddInterfaceRequired( "Input" );
    mtsInterfaceRequired* output = AddInterfaceRequired( "Output" );

    input->AddFunction( "GetPositionJoint", GetPositions );
    output->AddFunction( "SetTorqueJoint", SetTorques );

    mtsInterfaceProvided* prov = AddInterfaceProvided("PositionServer");
    if(prov){
      StateTable.AddData(cpos, "CartesianPosition");
      prov->AddCommandReadState( StateTable, cpos, "GetCartesianPosition");
    }
  }

  void Configure( const std::string& ){}
  void Startup(){}
  void Run(){
    ProcessQueuedCommands();
    
    // prmPositionJointGet q;
    GetPositions( q );

    vctFrame4x4<double> Rt;
    Rt = manipulator->ForwardKinematics(q.Position());

    vctMatRot3 rot(Rt.Rotation());
    vct3 tran(Rt.Translation());
    cpos.SetPosition(vctFrm3(rot,tran));

    //    std::cout << "q: " << q << std::endl;
//    std::cout << "Rt: \n" << Rt << std::endl;
//    std::cout << "cpos: \n" << cpos << std::endl;

    prmForceTorqueJointSet t;
    t.SetSize( 7 );
    t.ForceTorque().SetAll( 0.0 );
    SetTorques( t );

  }
  
  void Cleanup(){}

};

int main( int argc, char** argv ){

  mlockall(MCL_CURRENT | MCL_FUTURE);
  rt_task_shadow( NULL, "mtsWAMTest", 80, 0 );

  cmnLogger::SetMask( CMN_LOG_ALLOW_ALL );
  cmnLogger::SetMaskFunction( CMN_LOG_ALLOW_ALL );
  cmnLogger::SetMaskDefaultLog( CMN_LOG_ALLOW_ALL );

  if( argc != 3 ){
    std::cout << "Usage: " << argv[0] << " GCMIP rtcan[0-1]" << std::endl;
    return -1;
  }

  // mtsManager instance
  std::string process( "Server" );
  mtsManagerLocal* taskManager = NULL;

  try{ taskManager = mtsTaskManager::GetInstance( argv[1], process ); }
  catch( ... ){
    std::cerr << "Failed to connect to GCM: " << argv[1] << std::endl;
    taskManager = mtsManagerLocal::GetInstance();
  }

  // CAN bus
  osaRTSocketCAN can( argv[2], osaCANBus::RATE_1000 );

  if( can.Open() != osaCANBus::ESUCCESS ){
    CMN_LOG_RUN_ERROR << argv[0] << "Failed to open " << argv[2] << std::endl;
    return -1;
  }

  // mtsWAM
  mtsWAM WAM( "WAM", &can, osaWAM::WAM_7DOF, OSA_CPU4, 80 );
  WAM.Configure();
  WAM.SetPositions( vctDynamicVector<double>(7, 
					     0.0, -cmnPI_2, 0.0, cmnPI, 
					     0.0, 0.0, 0.0 ) );
  taskManager->AddComponent( &WAM );

  // WAMprobe
  WAMprobe probe;
  taskManager->AddComponent( &probe );

  // Connect interfaces
  taskManager->Connect( probe.GetName(), "Input",  WAM.GetName(),   "Output" );
  taskManager->Connect( probe.GetName(), "Output", WAM.GetName(),   "Input" );


  // Create & start components
  taskManager->CreateAll();
  taskManager->WaitForStateAll( mtsComponentState::READY );

  taskManager->StartAll();
  taskManager->WaitForStateAll( mtsComponentState::ACTIVE );

  std::cout << "ENTER to exit" << std::endl;
  cmnGetChar();

  taskManager->KillAll();
  taskManager->Cleanup();

  if( can.Close() != osaCANBus::ESUCCESS ){
    CMN_LOG_RUN_ERROR << argv[0] << "Failed to open " << argv[1] << std::endl;
    return -1;
  }

  return 0;
}
