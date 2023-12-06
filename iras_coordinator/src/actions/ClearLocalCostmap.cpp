#include <iras_coordinator/actions/ClearLocalCostmap.h>

void ClearLocalCostmap::on_send(std::shared_ptr<ClearLocalCostmapSrv::Request>)
{
}

bool ClearLocalCostmap::on_result(std::shared_future<std::shared_ptr<ClearLocalCostmapSrv::Response>>, std::shared_ptr<ClearLocalCostmapSrv::Request>)
{
    log("Local costmap cleared");
    return true;
}