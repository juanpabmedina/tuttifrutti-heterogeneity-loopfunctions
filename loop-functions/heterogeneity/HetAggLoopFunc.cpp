/**
  * @file <loop-functions/IcraLoopFunc.cpp>
  *
  * @author Antoine Ligot - <aligot@ulb.ac.be>
  *
  * @license MIT License
  */

#include "HetAggLoopFunc.h"

/****************************************/
/****************************************/

HetAggLoopFunction::HetAggLoopFunction() {
    m_unClock = 0;
    m_unStopTime = 0;
    m_unStopEdge = 2;
    m_unStopBox = 2;
    m_fObjectiveFunction = 0;
}

/****************************************/
/****************************************/

HetAggLoopFunction::HetAggLoopFunction(const HetAggLoopFunction& orig) {
}

/****************************************/
/****************************************/

HetAggLoopFunction::~HetAggLoopFunction() {}

/****************************************/
/****************************************/

void HetAggLoopFunction::Destroy() {

    m_tRobotStates.clear();
    RemoveArena();
}

/****************************************/
/****************************************/

void HetAggLoopFunction::Init(TConfigurationNode& t_tree) {

    CoreLoopFunctions::Init(t_tree);
    TConfigurationNode cParametersNode;
    try {
      cParametersNode = GetNode(t_tree, "params");
      GetNodeAttributeOrDefault(cParametersNode, "build_arena", m_bBuildArena, (bool) false);
      GetNodeAttributeOrDefault(cParametersNode, "number_edges", m_unNumberEdges, (UInt32) 3);
      GetNodeAttributeOrDefault(cParametersNode, "number_boxes_per_edge", m_unNumberBoxes, (UInt32) 1);
      GetNodeAttributeOrDefault(cParametersNode, "lenght_boxes", m_fLenghtBoxes, (Real) 0.25);
      GetNodeAttributeOrDefault(cParametersNode, "maximization", m_bMaximization, (bool) false);
    } catch(std::exception e) {
    }

    if (m_bBuildArena == true)
    {
        m_fDistributionRadius = GetArenaRadious();
        PositionArena();
    }

    InitRobotStates();
    ComputeWallVertices(m_unNumberEdges);
    m_pcArena->SetArenaColor(CColor::BLACK);

}

/****************************************/
/****************************************/

void HetAggLoopFunction::Reset() {
    CoreLoopFunctions::Reset();

    m_pcArena->SetArenaColor(CColor::BLACK);
    m_unClock = 0;
    m_unStopEdge = 2;
    m_unStopBox = 2;
    m_unStopTime = 0;
    m_fObjectiveFunction = 0;

    m_tRobotStates.clear();

    InitRobotStates();
}

/****************************************/
/****************************************/

void HetAggLoopFunction::PostStep() {

    m_unClock = GetSpace().GetSimulationClock();

    ArenaControl();
}

/****************************************/
/****************************************/

void HetAggLoopFunction::PostExperiment() {
    ScoreControl();
    if (m_bMaximization == true){
        LOG << -m_fObjectiveFunction << std::endl;
    }
    else {
        LOG << m_fObjectiveFunction << std::endl;
    }
}

/****************************************/
/****************************************/

Real HetAggLoopFunction::GetObjectiveFunction() {
    if (m_bMaximization == true){
        return -m_fObjectiveFunction;
    }
    else {
        return m_fObjectiveFunction;
    }
}

/****************************************/
/****************************************/

void HetAggLoopFunction::ArenaControl() {
    
    if (m_unClock == 1) {
        m_pcArena->SetBoxColor(4,4,CColor::RED);
        m_pcArena->SetBoxColor(1,3,CColor::RED);
        LOG << GetBoxPosition(3,2) << std::endl;
    }

    else if (m_unClock == 500) {
        ScoreControl();

        m_pcArena->SetBoxColor(4,4,CColor::BLACK);
        m_pcArena->SetBoxColor(1,3,CColor::BLACK);

        m_pcArena->SetBoxColor(4,3,CColor::YELLOW);
        m_pcArena->SetBoxColor(1,2,CColor::YELLOW);
    }

    else if (m_unClock == 1000) {
        ScoreControl();

        m_pcArena->SetBoxColor(4,3,CColor::BLACK);
        m_pcArena->SetBoxColor(1,2,CColor::BLACK);

        m_pcArena->SetBoxColor(4,2,CColor::GREEN);
        m_pcArena->SetBoxColor(1,1,CColor::GREEN);
    }

    else if (m_unClock == 1500) {
        ScoreControl();

        m_pcArena->SetBoxColor(4,2,CColor::BLACK);
        m_pcArena->SetBoxColor(1,1,CColor::BLACK);

        m_pcArena->SetBoxColor(4,1,CColor::BLUE);
        m_pcArena->SetBoxColor(1,6,CColor::BLUE); 
    }

    else if (m_unClock == 2000) {
        ScoreControl();

        m_pcArena->SetBoxColor(4,1,CColor::BLACK);
        m_pcArena->SetBoxColor(1,6,CColor::BLACK);

        m_pcArena->SetBoxColor(4,6,CColor::CYAN);
        m_pcArena->SetBoxColor(1,5,CColor::CYAN); 
    }

    else if (m_unClock == 2500) {
        ScoreControl();

        m_pcArena->SetBoxColor(4,6,CColor::BLACK);
        m_pcArena->SetBoxColor(1,5,CColor::BLACK);

        m_pcArena->SetBoxColor(1,4,CColor::MAGENTA);
        m_pcArena->SetBoxColor(4,5,CColor::MAGENTA);
    }

    return;
}

/****************************************/
/****************************************/

void HetAggLoopFunction::ComputeWallVertices(UInt32 unNumberEdges) {
    Real fRadious =  (m_fLenghtBoxes*m_unNumberBoxes) / (2 * Tan(CRadians::PI / m_unNumberEdges)) + 0.1;
    m_vecWallVertices.clear();

    Real fAngleStep = (2.0 * argos::CRadians::PI).GetValue() / unNumberEdges;

    // Shift angle by half a step to land on corners
    Real fAngleOffset = -0.5 * fAngleStep;

    for(UInt32 i = 0; i < unNumberEdges; ++i) {
        Real fAngle = i * fAngleStep + fAngleOffset;
        argos::CVector2 vertex(std::cos(fAngle), std::sin(fAngle));
        vertex *= fRadious;
        m_vecWallVertices.push_back(vertex);
    }
}


/****************************************/
/****************************************/

void HetAggLoopFunction::ScoreControl(){

    UpdateRobotPositions();
    Real fRadiusThreshold = 0.3;

    for (auto& it : m_tRobotStates) {
        const argos::CVector2& cRobotPos = it.second.cPosition;

        for (const argos::CVector2& cVertex : m_vecWallVertices) {
            
            if ((cRobotPos - cVertex).Length() <= fRadiusThreshold) {
                m_fObjectiveFunction += 1.0;
            }
        }
    }
    LOG << m_fObjectiveFunction << std::endl;
    

}

argos::CVector2 HetAggLoopFunction::GetBoxPosition(UInt32 unWallIdx, UInt32 unBoxIdx) {
    CWallEntity* pcWall = m_pcArena->GetWalls().at(unWallIdx);
    CBlockEntity* pcBlock = pcWall->GetBlocks().at(unBoxIdx);
    const argos::CVector3& cPos3D = pcBlock->GetEmbodiedEntity().GetOriginAnchor().Position;
    return argos::CVector2(cPos3D.GetX(), cPos3D.GetY());
}



/****************************************/
/****************************************/

Real HetAggLoopFunction::GetStopScore() {

    UpdateRobotPositions();

    Real unScore = 0;
    TRobotStateMap::iterator it;
    for (it = m_tRobotStates.begin(); it != m_tRobotStates.end(); ++it) {
        Real d = (it->second.cPosition - it->second.cLastPosition).Length();
        if (d > 0.0005)
            unScore+=1;
    }

    return unScore;
}

/****************************************/
/****************************************/

Real HetAggLoopFunction::GetMoveScore() {

    UpdateRobotPositions();

    Real unScore = 0;
    TRobotStateMap::iterator it;
    for (it = m_tRobotStates.begin(); it != m_tRobotStates.end(); ++it) {
        Real d = (it->second.cPosition - it->second.cLastPosition).Length();
        if (d <= 0.0005)
            unScore+=1;
    }

    return unScore;
}

/****************************************/
/****************************************/

argos::CColor HetAggLoopFunction::GetFloorColor(const argos::CVector2& c_position_on_plane) {
    

    Real fThreshold = 0.3;

    for(const argos::CVector2& cVertex : m_vecWallVertices) {
        if ((c_position_on_plane - cVertex).Length() <= fThreshold) {
            return argos::CColor::GRAY40;
        }
    }

    return argos::CColor::GRAY50;
}

/****************************************/
/****************************************/

void HetAggLoopFunction::UpdateRobotPositions() {
    CSpace::TMapPerType& tEpuckMap = GetSpace().GetEntitiesByType("epuck");
    CVector2 cEpuckPosition(0,0);
    for (CSpace::TMapPerType::iterator it = tEpuckMap.begin(); it != tEpuckMap.end(); ++it) {
        CEPuckEntity* pcEpuck = any_cast<CEPuckEntity*>(it->second);
        cEpuckPosition.Set(pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                           pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

        m_tRobotStates[pcEpuck].cLastPosition = m_tRobotStates[pcEpuck].cPosition;
        m_tRobotStates[pcEpuck].cPosition = cEpuckPosition;
    }
}

/****************************************/
/****************************************/

void HetAggLoopFunction::InitRobotStates() {

    CSpace::TMapPerType& tEpuckMap = GetSpace().GetEntitiesByType("epuck");
    CVector2 cEpuckPosition(0,0);
    for (CSpace::TMapPerType::iterator it = tEpuckMap.begin(); it != tEpuckMap.end(); ++it) {
        CEPuckEntity* pcEpuck = any_cast<CEPuckEntity*>(it->second);
        cEpuckPosition.Set(pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                           pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

        m_tRobotStates[pcEpuck].cLastPosition = cEpuckPosition;
        m_tRobotStates[pcEpuck].cPosition = cEpuckPosition;
        m_tRobotStates[pcEpuck].unItem = 0;
    }
}

/****************************************/
/****************************************/

CVector3 HetAggLoopFunction::GetRandomPosition() {
  Real temp;
  Real a = m_pcRng->Uniform(CRange<Real>(0.0f, 1.0f));
  Real b = m_pcRng->Uniform(CRange<Real>(0.0f, 1.0f));
  Real c = m_pcRng->Uniform(CRange<Real>(-1.0f, 1.0f));
  Real d = m_pcRng->Uniform(CRange<Real>(-1.0f, 1.0f));
  // If b < a, swap them
  if (b < a) {
    temp = a;
    a = b;
    b = temp;
  }
  m_fDistributionRadius = 0.4;
  Real fPosX = (c * m_fDistributionRadius / 2) + m_fDistributionRadius * cos(2 * -CRadians::PI_OVER_TWO .GetValue() * (a/b));
  Real fPosY = -0.20 + (d * m_fDistributionRadius / 2) + m_fDistributionRadius * sin(2 * -CRadians::PI_OVER_TWO.GetValue() * (a/b));

  return CVector3(fPosX, fPosY, 0);
}

/****************************************/
/****************************************/

UInt32 HetAggLoopFunction::GetRandomTime(UInt32 unMin, UInt32 unMax) {
  UInt32 unStopAt = m_pcRng->Uniform(CRange<UInt32>(unMin, unMax));
  return unStopAt;

}

/****************************************/
/****************************************/

void HetAggLoopFunction::PositionArena() {
  CArenaEntity* pcArena;
  /*
    pcArena = new CArenaEntity("arena",
                               CVector3(0,0,0),
                               CQuaternion().FromEulerAngles(CRadians::ZERO,CRadians::ZERO,CRadians::ZERO), // TODO
                               CVector3(0.01,m_fLenghtBoxes,0.1),
                               "leds",
                               m_unNumberBoxes,
                               m_unNumberEdges,
                               0.017f,
                               1.0f);   */ // arena with 12 leds per block

  pcArena = new CArenaEntity("arena",
                             CVector3(0,0,0),
                             CQuaternion().FromEulerAngles(CRadians::ZERO,CRadians::ZERO,CRadians::ZERO), // TODO
                             CVector3(0.01,m_fLenghtBoxes,0.1),
                             "leds",
                             m_unNumberBoxes,
                             m_unNumberEdges,
                             0.125f,
                             1.0f);

  AddEntity(*pcArena);
  m_pcArena = pcArena;
}

/****************************************/
/****************************************/

void HetAggLoopFunction::RemoveArena() {
    std::ostringstream id;
    id << "arena";
    RemoveEntity(id.str().c_str());
}

/****************************************/
/****************************************/

Real HetAggLoopFunction::GetArenaRadious() {

    Real fRadious;
    fRadious =  (m_fLenghtBoxes*m_unNumberBoxes) / (2 * Tan(CRadians::PI / m_unNumberEdges));
    //fRadious = fRadious - 0.10; // Avoids to place robots close the walls.
    fRadious = fRadious - 0.65; // Reduced cluster at the begining

    return fRadious;
}

/****************************************/
/****************************************/

bool HetAggLoopFunction::IsEven(UInt32 unNumber) {
    bool even;
    if((unNumber%2)==0)
       even = true;
    else
       even = false;

    return even;
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(HetAggLoopFunction, "het_agg_loop_function");
