#include <iras_coordinator/actions/ClearGlobalCostmap.h>

void ClearGlobalCostmap::on_send(std::shared_ptr<ClearGlobalCostmapSrv::Request>)
{
}

bool ClearGlobalCostmap::on_result(std::shared_future<std::shared_ptr<ClearGlobalCostmapSrv::Response>>, std::shared_ptr<ClearGlobalCostmapSrv::Request>)
{
    log("Global costmap cleared");
    return true;
}