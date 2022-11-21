/*********************************************************************************
 * @file        data_association.h
 * @brief       数据关联（SensorObject与TrackObject）
 * @details
 * @author      zhoucw
 * @date        2022/10/27
 * @copyright   Copyright (c) 2022 Gohigh Fusion Division.
 ********************************************************************************/
#ifndef __DATA_ASSOCIATION_H__
#define __DATA_ASSOCIATION_H__

#include <cstdint>
#include <vector>
#include <Eigen/Dense>

#include "infrastructure/hungarian/hungarian.h"

struct SensorFrame;
struct TrackMatchPair;

// 定义 TrackID 和 index 之间的映射关系
struct TrackObjectIDMap
{
	std::vector<uint32_t> vecTrackIDIndex;
	uint32_t nObjectIDIndex;
};

class DataAssociation
{
public:
	void Associate(const SensorFrame& frame, const std::vector<uint32_t>& sensor_track_index_list);
	const std::vector<TrackMatchPair>& GetMatchResult();
	const std::vector<uint32_t>& GetUnmatchedTracks();
	const std::vector<uint32_t>& GetUnmatchedObjects();

private:
	/**
	 * @brief 匹配质量检验（校验 match_result 集合中的 ID，将校验不通过的分别存放在 unmatched 集合中）
	 * @param const SensorFrame & frame
	 * @return
	 */
	void MatchQualityCheck(const SensorFrame& frame);

	bool CheckMahalDist(const SensorObject& xSensorObject, const FusionTrack& xFusionTrack);

	void DoKahnMunkresMatch(const SensorFrame& frame);
	void DoIdAssign(const SensorFrame& frame, const std::vector<uint32_t>& sensor_track_index_list);

private:
	std::vector<TrackMatchPair>     match_result;			// 存放匹配上的节点数据
	std::vector<uint32_t>           unmatched_tracks_index; // 存放未匹配上的 track 数据
	std::vector<uint32_t>           unmatched_objects_index;// 存放未匹配上的 object 数据
};

#endif // !__DATA_ASSOCIATION_H__
