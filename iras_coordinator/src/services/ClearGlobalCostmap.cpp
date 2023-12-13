#include <iras_coordinator/services/ClearGlobalCostmap.h>

std::string ClearGlobalCostmap::ros2_service_name()
{
    return "/global_costmap/clear_entirely_global_costmap";
}

void ClearGlobalCostmap::on_send(std::shared_ptr<ClearGlobalCostmapSrv::Request>)
{
}

bool ClearGlobalCostmap::on_result(std::shared_future<std::shared_ptr<ClearGlobalCostmapSrv::Response>>, std::shared_ptr<ClearGlobalCostmapSrv::Request>)
{
    log("Global costmap cleared");
    return true;
}