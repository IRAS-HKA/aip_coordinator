#include <iras_coordinator/services/ClearLocalCostmap.h>

std::string ClearLocalCostmap::ros2_service_name()
{
    return "/local_costmap/clear_entirely_local_costmap";
}

void ClearLocalCostmap::on_send(std::shared_ptr<ClearLocalCostmapSrv::Request>)
{
}

bool ClearLocalCostmap::on_result(std::shared_future<std::shared_ptr<ClearLocalCostmapSrv::Response>>, std::shared_ptr<ClearLocalCostmapSrv::Request>)
{
    log("Local costmap cleared");
    return true;
}