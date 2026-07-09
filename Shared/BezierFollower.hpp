#pragma once
#include "Map.hpp"
#include <cmath>

// 2D 벡터 구조체 (간단히 정의)
struct Vector2 { float x, z; };

class BezierFollower
{
public:
    // 1. 현재 진행률(t)에 따른 위치(Position) 계산 (3차 베지어 곡선 공식)
    static Vector2 Evaluate(const MapNode& fromNode, const MapNode& toNode, const MapLink& link, float t)
    {
        // t는 0.0(출발점) ~ 1.0(도착점) 사이의 값
        if (t < 0.0f) t = 0.0f;
        if (t > 1.0f) t = 1.0f;

        if(link.m_Type != 1) // 직선 링크라면 단순 선형 보간
        {
            Vector2 p;
            p.x = fromNode.m_PosX + (toNode.m_PosX - fromNode.m_PosX) * t;
            p.z = fromNode.m_PosZ + (toNode.m_PosZ - fromNode.m_PosZ) * t;
            return p;
        }

        float u = 1.0f - t;
        float tt = t * t;
        float uu = u * u;
        float uuu = uu * u;
        float ttt = tt * t;

        Vector2 p;
        // P0: 출발지
        // P1: 제어점1 (cx1, cz1)
        // P2: 제어점2 (cx2, cz2)
        // P3: 도착지
        p.x = uuu * fromNode.m_PosX + 3 * uu * t * link.m_CX1 + 3 * u * tt * link.m_CX2 + ttt * toNode.m_PosX;
        p.z = uuu * fromNode.m_PosZ + 3 * uu * t * link.m_CZ1 + 3 * u * tt * link.m_CZ2 + ttt * toNode.m_PosZ;

        return p;
    }

    // 2. 현재 진행률(t)에 따른 방향(Heading, 라디안) 계산 (베지어 곡선의 1차 미분)
    static float Heading(const MapNode& fromNode, const MapNode& toNode, const MapLink& link, float t)
    {
        if (t < 0.0f) t = 0.0f;
        if (t > 1.0f) t = 1.0f;

        float u = 1.0f - t;

        // 미분된 x, z 벡터 (접선 벡터)
        float dx = 3 * u * u * (link.m_CX1 - fromNode.m_PosX) +
                   6 * u * t * (link.m_CX2 - link.m_CX1) +
                   3 * t * t * (toNode.m_PosX - link.m_CX2);

        float dz = 3 * u * u * (link.m_CZ1 - fromNode.m_PosZ) +
                   6 * u * t * (link.m_CZ2 - link.m_CZ1) +
                   3 * t * t * (toNode.m_PosZ - link.m_CZ2);

        // atan2를 이용해 접선 벡터의 각도를 라디안으로 반환
        return std::atan2(dz, dx);
    }
};