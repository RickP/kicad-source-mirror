/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * Copyright The KiCad Developers, see AUTHORS.txt for contributors.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 3
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you may find one here:
 * http://www.gnu.org/licenses/gpl-3.0.html
 * or you may search the http://www.gnu.org website for the version 3 license,
 * or you may write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA
 */

#include "zone_utils.h"

#include <board.h>
#include <board_commit.h>
#include <footprint.h>
#include <pad.h>
#include <pcb_group.h>
#include <pcb_track.h>
#include <thread_pool.h>
#include <zone.h>
#include <geometry/shape_poly_set.h>

#include <algorithm>
#include <climits>
#include <cmath>
#include <future>
#include <map>
#include <optional>
#include <unordered_map>
#include <unordered_set>


namespace
{

constexpr int MAX_STITCHING_VIAS = 10000;

const wxString& stitchingGroupPrefix()
{
    static const wxString prefix = wxS( "__zone_via_stitching_" );
    return prefix;
}


wxString stitchingGroupPrefix( const ZONE& aZone )
{
    return stitchingGroupPrefix() + aZone.m_Uuid.AsString();
}


const wxChar* modeCode( ZONE_VIA_STITCHING_MODE aMode )
{
    switch( aMode )
    {
    case ZONE_VIA_STITCHING_MODE::GRID:  return wxS( "g" );
    case ZONE_VIA_STITCHING_MODE::FENCE: return wxS( "f" );
    case ZONE_VIA_STITCHING_MODE::NONE:  return wxS( "n" );
    }

    return wxS( "n" );
}


const wxChar* edgeModeCode( ZONE_VIA_STITCHING_EDGE_MODE aMode )
{
    switch( aMode )
    {
    case ZONE_VIA_STITCHING_EDGE_MODE::ALL:     return wxS( "a" );
    case ZONE_VIA_STITCHING_EDGE_MODE::OUTSIDE: return wxS( "o" );
    case ZONE_VIA_STITCHING_EDGE_MODE::INSIDE:  return wxS( "i" );
    }

    return wxS( "a" );
}


wxString stitchingGroupName( const ZONE& aZone )
{
    return wxString::Format( wxS( "%s;v=1;m=%s;e=%s;p=%d;o=%d;d=%d;r=%d" ),
                             stitchingGroupPrefix( aZone ).c_str(),
                             modeCode( aZone.GetViaStitchingMode() ),
                             edgeModeCode( aZone.GetViaStitchingEdgeMode() ),
                             aZone.GetViaStitchingPitch(),
                             aZone.GetViaStitchingOffset(),
                             aZone.GetViaStitchingDiameter(),
                             aZone.GetViaStitchingDrill() );
}


bool isSameZoneStitchingGroup( const EDA_GROUP* aGroup, const ZONE& aZone )
{
    return aGroup && aGroup->GetName().StartsWith( stitchingGroupPrefix( aZone ) );
}


PCB_GROUP* findStitchingGroup( BOARD* aBoard, const ZONE& aZone )
{
    if( !aBoard )
        return nullptr;

    for( PCB_GROUP* group : aBoard->Groups() )
    {
        if( isSameZoneStitchingGroup( group, aZone ) )
            return group;
    }

    return nullptr;
}


struct STITCHING_GROUP_SETTINGS
{
    wxString                       uuid;
    bool                           hasSettings = false;
    ZONE_VIA_STITCHING_MODE        mode = ZONE_VIA_STITCHING_MODE::NONE;
    ZONE_VIA_STITCHING_EDGE_MODE   edgeMode = ZONE_VIA_STITCHING_EDGE_MODE::ALL;
    int                            pitch = 0;
    int                            offset = 0;
    int                            diameter = 0;
    int                            drill = 0;
};


bool parseStitchingGroupName( const wxString& aName, STITCHING_GROUP_SETTINGS& aSettings )
{
    const wxString& prefix = stitchingGroupPrefix();

    if( !aName.StartsWith( prefix ) )
        return false;

    wxString remaining = aName.Mid( prefix.length() );
    wxString fields;
    int      fieldStart = remaining.Find( wxS( ';' ) );

    if( fieldStart == wxNOT_FOUND )
    {
        aSettings.uuid = remaining;
        return !aSettings.uuid.IsEmpty();
    }

    aSettings.uuid = remaining.Left( fieldStart );
    fields = remaining.Mid( fieldStart + 1 );

    while( !fields.IsEmpty() )
    {
        wxString field = fields.BeforeFirst( ';' );

        if( field.length() == fields.length() )
            fields.clear();
        else
            fields = fields.Mid( field.length() + 1 );

        wxString key = field.BeforeFirst( '=' );
        wxString value = field.AfterFirst( '=' );

        if( key == wxS( "m" ) )
        {
            if( value == wxS( "g" ) )
                aSettings.mode = ZONE_VIA_STITCHING_MODE::GRID;
            else if( value == wxS( "f" ) )
                aSettings.mode = ZONE_VIA_STITCHING_MODE::FENCE;
            else
                aSettings.mode = ZONE_VIA_STITCHING_MODE::NONE;

            aSettings.hasSettings = true;
        }
        else if( key == wxS( "e" ) )
        {
            if( value == wxS( "o" ) )
                aSettings.edgeMode = ZONE_VIA_STITCHING_EDGE_MODE::OUTSIDE;
            else if( value == wxS( "i" ) )
                aSettings.edgeMode = ZONE_VIA_STITCHING_EDGE_MODE::INSIDE;
            else
                aSettings.edgeMode = ZONE_VIA_STITCHING_EDGE_MODE::ALL;

            aSettings.hasSettings = true;
        }
        else if( key == wxS( "p" ) )
        {
            aSettings.pitch = wxAtoi( value );
            aSettings.hasSettings = true;
        }
        else if( key == wxS( "o" ) )
        {
            aSettings.offset = wxAtoi( value );
            aSettings.hasSettings = true;
        }
        else if( key == wxS( "d" ) )
        {
            aSettings.diameter = wxAtoi( value );
            aSettings.hasSettings = true;
        }
        else if( key == wxS( "r" ) )
        {
            aSettings.drill = wxAtoi( value );
            aSettings.hasSettings = true;
        }
    }

    return !aSettings.uuid.IsEmpty();
}


bool isStitchingViaForZone( const PCB_VIA& aVia, const ZONE& aZone )
{
    return isSameZoneStitchingGroup( aVia.GetParentGroup(), aZone );
}


bool isDuplicateCandidatePoint( const std::vector<VECTOR2I>& aPoints, const VECTOR2I& aPoint,
                                int aMinDistance )
{
    int64_t minDistSq = static_cast<int64_t>( aMinDistance ) * aMinDistance;

    for( const VECTOR2I& point : aPoints )
    {
        if( ( point - aPoint ).SquaredEuclideanNorm() < minDistSq )
            return true;
    }

    return false;
}


bool addCandidatePoint( const VECTOR2I& aPoint, int aDuplicateDistance,
                        std::vector<VECTOR2I>& aPoints )
{
    if( aDuplicateDistance > 0 && isDuplicateCandidatePoint( aPoints, aPoint, aDuplicateDistance ) )
        return false;

    if( (int) aPoints.size() >= MAX_STITCHING_VIAS )
        return false;

    aPoints.push_back( aPoint );
    return true;
}


SHAPE_POLY_SET deflatedViaCenterAllowedArea( const SHAPE_POLY_SET& aPolySet, int aViaRadius,
                                             int aMaxError )
{
    SHAPE_POLY_SET allowed = aPolySet.CloneDropTriangulation();

    if( !allowed.IsEmpty() )
    {
        allowed.Simplify();
        allowed.Deflate( aViaRadius, CORNER_STRATEGY::ROUND_ALL_CORNERS, aMaxError );
    }

    return allowed;
}


void collectGridStitchingPoints( const ZONE& aZone, std::vector<VECTOR2I>& aPoints )
{
    int pitch = aZone.GetViaStitchingPitch();

    if( pitch <= 0 )
        return;

    int   clearance = aZone.GetViaStitchingDiameter() / 2 + aZone.GetViaStitchingOffset();
    BOX2I bbox = aZone.Outline()->BBox();

    int left = bbox.GetLeft() + clearance;
    int right = bbox.GetRight() - clearance;
    int top = bbox.GetTop() + clearance;
    int bottom = bbox.GetBottom() - clearance;

    int halfPitch = pitch / 2;
    int row = 0;

    for( int y = top; y <= bottom && (int) aPoints.size() < MAX_STITCHING_VIAS; y += pitch, ++row )
    {
        int rowLeft = left + ( row % 2 ? halfPitch : 0 );

        for( int x = rowLeft; x <= right && (int) aPoints.size() < MAX_STITCHING_VIAS; x += pitch )
            addCandidatePoint( VECTOR2I( x, y ), 0, aPoints );
    }
}


bool isNearZoneOuterOutline( const ZONE& aZone, const VECTOR2I& aPoint, int aFenceOffset )
{
    const SHAPE_POLY_SET* outline = aZone.Outline();

    if( !outline || outline->IsEmpty() )
        return false;

    int tolerance = std::max( aZone.GetMaxError() * 2, std::max( 1, aFenceOffset / 4 ) );
    int minDistance = INT_MAX;

    for( int ii = 0; ii < outline->OutlineCount(); ++ii )
        minDistance = std::min( minDistance, outline->COutline( ii ).Distance( aPoint, true ) );

    return std::abs( minDistance - aFenceOffset ) <= tolerance;
}


bool fencePointMatchesEdgeMode( const ZONE& aZone, const VECTOR2I& aPoint, int aFenceOffset )
{
    ZONE_VIA_STITCHING_EDGE_MODE edgeMode = aZone.GetViaStitchingEdgeMode();

    if( edgeMode == ZONE_VIA_STITCHING_EDGE_MODE::ALL )
        return true;

    bool nearOuterOutline = isNearZoneOuterOutline( aZone, aPoint, aFenceOffset );

    if( edgeMode == ZONE_VIA_STITCHING_EDGE_MODE::OUTSIDE )
        return nearOuterOutline;

    return !nearOuterOutline;
}


void collectFencePointsFromChain( const ZONE& aZone, const SHAPE_LINE_CHAIN& aChain,
                                  const SHAPE_POLY_SET& aAllowedArea, int aFenceOffset,
                                  std::vector<VECTOR2I>& aPoints )
{
    int pitch = aZone.GetViaStitchingPitch();

    if( pitch <= 0 )
        return;

    int viaRadius = aZone.GetViaStitchingDiameter() / 2;
    int duplicateDistance = std::max( viaRadius * 2, pitch / 2 );
    int length = static_cast<int>( aChain.Length() );

    if( length <= 0 )
        return;

    int count = std::max( 1, static_cast<int>( std::floor( static_cast<double>( length ) / pitch ) ) );

    for( int ii = 0; ii < count && (int) aPoints.size() < MAX_STITCHING_VIAS; ++ii )
    {
        int      pathLength = KiROUND( ( ii + 0.5 ) * static_cast<double>( length ) / count );
        VECTOR2I point = aChain.PointAlong( pathLength % length );

        if( aAllowedArea.Contains( point )
            && fencePointMatchesEdgeMode( aZone, point, aFenceOffset ) )
        {
            addCandidatePoint( point, duplicateDistance, aPoints );
        }
    }
}


void collectFencePointsFromPolySet( const ZONE& aZone, const SHAPE_POLY_SET& aPolySet,
                                    std::vector<VECTOR2I>& aPoints )
{
    SHAPE_POLY_SET fenceGeometry = aPolySet.CloneDropTriangulation();

    if( fenceGeometry.IsEmpty() )
        return;

    fenceGeometry.Simplify();

    int viaRadius = aZone.GetViaStitchingDiameter() / 2;
    int fenceOffset = viaRadius + aZone.GetViaStitchingOffset();
    int minFenceContourLength = std::max( aZone.GetViaStitchingPitch() * 2, fenceOffset * 4 );

    SHAPE_POLY_SET allowedArea = fenceGeometry;
    allowedArea.Deflate( viaRadius, CORNER_STRATEGY::ROUND_ALL_CORNERS, aZone.GetMaxError() );

    SHAPE_POLY_SET fencePath = fenceGeometry;
    fencePath.Deflate( fenceOffset, CORNER_STRATEGY::ROUND_ALL_CORNERS, aZone.GetMaxError() );
    fencePath.Simplify();

    if( fencePath.IsEmpty() )
        return;

    for( int ii = 0; ii < fencePath.OutlineCount()
              && (int) aPoints.size() < MAX_STITCHING_VIAS; ++ii )
    {
        if( fencePath.COutline( ii ).Length() >= minFenceContourLength )
            collectFencePointsFromChain( aZone, fencePath.COutline( ii ), allowedArea, fenceOffset,
                                         aPoints );

        for( int jj = 0; jj < fencePath.HoleCount( ii )
                  && (int) aPoints.size() < MAX_STITCHING_VIAS; ++jj )
        {
            if( fencePath.CHole( ii, jj ).Length() >= minFenceContourLength )
            {
                collectFencePointsFromChain( aZone, fencePath.CHole( ii, jj ), allowedArea,
                                             fenceOffset, aPoints );
            }
        }
    }
}


void collectFenceStitchingPoints( BOARD* aBoard, const ZONE& aZone, std::vector<VECTOR2I>& aPoints )
{
    bool usedFillGeometry = false;
    LSET zoneCopperLayers = aZone.GetLayerSet() & LSET::AllCuMask();

    zoneCopperLayers.RunOnLayers(
            [&]( PCB_LAYER_ID layer )
            {
                if( aZone.HasFilledPolysForLayer( layer ) )
                {
                    usedFillGeometry = true;
                    collectFencePointsFromPolySet( aZone, *aZone.GetFilledPolysList( layer ), aPoints );
                }
            } );

    if( !usedFillGeometry )
        collectFencePointsFromPolySet( aZone, *aZone.Outline(), aPoints );
}


void collectViaStitchingPoints( BOARD* aBoard, const ZONE& aZone, std::vector<VECTOR2I>& aPoints )
{
    if( aZone.GetViaStitchingMode() == ZONE_VIA_STITCHING_MODE::GRID )
        collectGridStitchingPoints( aZone, aPoints );
    else if( aZone.GetViaStitchingMode() == ZONE_VIA_STITCHING_MODE::FENCE )
        collectFenceStitchingPoints( aBoard, aZone, aPoints );
}


bool isZoneStitchingCandidate( const PCB_VIA& aVia, const ZONE& aZone )
{
    PCB_LAYER_ID top;
    PCB_LAYER_ID bottom;

    aVia.LayerPair( &top, &bottom );

    return aVia.GetIsFree() && aVia.GetViaType() == VIATYPE::THROUGH
           && aVia.GetNetCode() == aZone.GetNetCode()
           && aVia.GetWidth( PADSTACK::ALL_LAYERS ) == aZone.GetViaStitchingDiameter()
           && aVia.GetDrillValue() == aZone.GetViaStitchingDrill()
           && ( ( top == F_Cu && bottom == B_Cu ) || ( top == B_Cu && bottom == F_Cu ) );
}


class STITCHING_VALIDATOR
{
public:
    STITCHING_VALIDATOR( BOARD* aBoard, const ZONE& aSourceZone, int aViaDiameter ) :
            m_board( aBoard ),
            m_sourceZone( aSourceZone ),
            m_viaRadius( aViaDiameter / 2 )
    {
    }

    bool IsValid( const PCB_VIA& aVia )
    {
        return sourceZoneContainsVia( aVia )
               && connectedSameNetLayerCount( aVia ) >= 2
               && !collidesWithPadsOrTracks( aVia )
               && !collidesWithViaKeepout( aVia );
    }

private:
    const SHAPE_POLY_SET& viaCenterAllowedArea( const ZONE& aZone, PCB_LAYER_ID aLayer )
    {
        std::pair<const ZONE*, PCB_LAYER_ID> key( &aZone, aLayer );
        auto                                it = m_viaCenterAllowedAreas.find( key );

        if( it != m_viaCenterAllowedAreas.end() )
            return it->second;

        SHAPE_POLY_SET allowed;

        if( aZone.HasFilledPolysForLayer( aLayer ) )
        {
            std::shared_ptr<SHAPE_POLY_SET> fill = aZone.GetFilledPolysList( aLayer );

            if( fill && !fill->IsEmpty() )
                allowed = deflatedViaCenterAllowedArea( *fill, m_viaRadius, aZone.GetMaxError() );
        }
        else
        {
            allowed = deflatedViaCenterAllowedArea( *aZone.Outline(), m_viaRadius,
                                                    aZone.GetMaxError() );
        }

        return m_viaCenterAllowedAreas.emplace( key, allowed ).first->second;
    }

    bool zoneLayerContainsVia( const ZONE& aZone, PCB_LAYER_ID aLayer, const PCB_VIA& aVia )
    {
        const SHAPE_POLY_SET& allowed = viaCenterAllowedArea( aZone, aLayer );
        return !allowed.IsEmpty() && allowed.Contains( aVia.GetPosition() );
    }

    bool sourceZoneContainsVia( const PCB_VIA& aVia )
    {
        bool containsVia = false;
        LSET zoneCopperLayers = m_sourceZone.GetLayerSet() & LSET::AllCuMask();

        zoneCopperLayers.RunOnLayers(
                [&]( PCB_LAYER_ID layer )
                {
                    if( zoneLayerContainsVia( m_sourceZone, layer, aVia ) )
                        containsVia = true;
                } );

        return containsVia;
    }

    int connectedSameNetLayerCount( const PCB_VIA& aVia )
    {
        LSET connectedLayers;

        auto testZone =
                [&]( const ZONE& zone )
                {
                    if( zone.GetIsRuleArea() || zone.IsTeardropArea() || !zone.IsOnCopperLayer()
                        || zone.GetNetCode() != m_sourceZone.GetNetCode() )
                    {
                        return;
                    }

                    LSET zoneCopperLayers = zone.GetLayerSet() & LSET::AllCuMask();

                    zoneCopperLayers.RunOnLayers(
                            [&]( PCB_LAYER_ID layer )
                            {
                                if( zoneLayerContainsVia( zone, layer, aVia ) )
                                    connectedLayers.set( layer );
                            } );
                };

        testZone( m_sourceZone );

        if( m_board )
        {
            for( ZONE* zone : m_board->Zones() )
            {
                if( zone != &m_sourceZone )
                    testZone( *zone );
            }
        }

        return connectedLayers.count();
    }

    bool collidesWithPadsOrTracks( const PCB_VIA& aVia )
    {
        for( PCB_TRACK* track : m_board->Tracks() )
        {
            if( track == &aVia )
                continue;

            if( PCB_VIA* existingVia = dyn_cast<PCB_VIA*>( track ) )
            {
                if( isStitchingViaForZone( *existingVia, m_sourceZone ) )
                    continue;

            }

            LSET trackCopperLayers = track->GetLayerSet() & LSET::AllCuMask();
            bool collides = false;

            trackCopperLayers.RunOnLayers(
                    [&]( PCB_LAYER_ID layer )
                    {
                        std::shared_ptr<SHAPE> viaShape =
                                aVia.GetEffectiveShape( layer, FLASHING::ALWAYS_FLASHED );
                        std::shared_ptr<SHAPE> trackShape =
                                track->GetEffectiveShape( layer, FLASHING::ALWAYS_FLASHED );

                        if( viaShape && trackShape && viaShape->Collide( trackShape.get() ) )
                            collides = true;
                    } );

            if( collides )
                return true;
        }

        for( FOOTPRINT* footprint : m_board->Footprints() )
        {
            for( PAD* pad : footprint->Pads() )
            {
                LSET padCopperLayers = pad->GetLayerSet() & LSET::AllCuMask();
                bool collides = false;

                padCopperLayers.RunOnLayers(
                        [&]( PCB_LAYER_ID layer )
                        {
                            std::shared_ptr<SHAPE> viaShape =
                                    aVia.GetEffectiveShape( layer, FLASHING::ALWAYS_FLASHED );
                            std::shared_ptr<SHAPE> padShape =
                                    pad->GetEffectiveShape( layer, FLASHING::ALWAYS_FLASHED );

                            if( viaShape && padShape && viaShape->Collide( padShape.get() ) )
                                collides = true;
                        } );

                if( collides )
                    return true;
            }
        }

        return false;
    }

    bool collidesWithViaKeepout( const PCB_VIA& aVia )
    {
        for( ZONE* zone : m_board->Zones() )
        {
            if( !zone->GetIsRuleArea() || !zone->GetDoNotAllowVias()
                || !( zone->GetLayerSet() & LSET::AllCuMask() ).any() )
            {
                continue;
            }

            LSET keepoutCopperLayers = zone->GetLayerSet() & LSET::AllCuMask();
            bool collides = false;

            keepoutCopperLayers.RunOnLayers(
                    [&]( PCB_LAYER_ID layer )
                    {
                        std::shared_ptr<SHAPE> viaShape =
                                aVia.GetEffectiveShape( layer, FLASHING::ALWAYS_FLASHED );

                        if( viaShape && zone->Outline()->Collide( viaShape.get(), 0 ) )
                            collides = true;
                    } );

            if( collides )
                return true;
        }

        return false;
    }

private:
    BOARD*                                             m_board;
    const ZONE&                                        m_sourceZone;
    int                                                m_viaRadius;
    std::map<std::pair<const ZONE*, PCB_LAYER_ID>, SHAPE_POLY_SET>
            m_viaCenterAllowedAreas;
};


bool hasMatchingViaAtPoint( BOARD* aBoard, const ZONE& aZone, const VECTOR2I& aPoint )
{
    for( PCB_TRACK* track : aBoard->Tracks() )
    {
        PCB_VIA* via = dyn_cast<PCB_VIA*>( track );

        if( via && via->GetPosition() == aPoint && isZoneStitchingCandidate( *via, aZone ) )
            return true;
    }

    return false;
}


void addZoneViaStitching( BOARD_COMMIT& aCommit, BOARD* aBoard, const ZONE& aZone,
                          bool aSkipExisting )
{
    if( !aBoard || aZone.GetIsRuleArea() || aZone.IsTeardropArea() || !aZone.IsOnCopperLayer()
        || aZone.GetNetCode() <= 0 || aZone.GetViaStitchingMode() == ZONE_VIA_STITCHING_MODE::NONE )
    {
        return;
    }

    std::vector<VECTOR2I> points;
    collectViaStitchingPoints( aBoard, aZone, points );
    std::vector<PCB_VIA*> vias;
    STITCHING_VALIDATOR   validator( aBoard, aZone, aZone.GetViaStitchingDiameter() );

    for( const VECTOR2I& point : points )
    {
        if( aSkipExisting && hasMatchingViaAtPoint( aBoard, aZone, point ) )
            continue;

        PCB_VIA* via = new PCB_VIA( aBoard );
        via->SetPosition( point );
        via->SetNetCode( aZone.GetNetCode() );
        via->SetIsFree( true );
        via->SetViaType( VIATYPE::THROUGH );
        via->SetLayerPair( B_Cu, F_Cu );
        via->SetWidth( PADSTACK::ALL_LAYERS, aZone.GetViaStitchingDiameter() );
        via->SetDrill( aZone.GetViaStitchingDrill() );

        if( !validator.IsValid( *via ) )
        {
            delete via;
            continue;
        }

        vias.push_back( via );
    }

    PCB_GROUP* group = new PCB_GROUP( aBoard );
    group->SetName( stitchingGroupName( aZone ) );

    for( PCB_VIA* via : vias )
    {
        group->AddItem( via );
        aCommit.Add( via );
    }

    aCommit.Add( group );
}

} // namespace


void RestoreZoneViaStitchingSettings( BOARD* aBoard )
{
    if( !aBoard )
        return;

    for( PCB_GROUP* group : aBoard->Groups() )
    {
        STITCHING_GROUP_SETTINGS settings;

        if( !parseStitchingGroupName( group->GetName(), settings ) )
            continue;

        for( ZONE* zone : aBoard->Zones() )
        {
            if( zone->m_Uuid.AsString() != settings.uuid )
                continue;

            if( settings.hasSettings )
            {
                zone->SetViaStitchingMode( settings.mode );
                zone->SetViaStitchingEdgeMode( settings.edgeMode );

                if( settings.pitch > 0 )
                    zone->SetViaStitchingPitch( settings.pitch );

                if( settings.offset >= 0 )
                    zone->SetViaStitchingOffset( settings.offset );

                if( settings.diameter > 0 )
                    zone->SetViaStitchingDiameter( settings.diameter );

                if( settings.drill > 0 )
                    zone->SetViaStitchingDrill( settings.drill );
            }

            if( zone->GetViaStitchingMode() != ZONE_VIA_STITCHING_MODE::NONE )
                group->SetName( stitchingGroupName( *zone ) );

            break;
        }
    }
}


bool IsZoneViaStitchingVia( const PCB_VIA& aVia )
{
    EDA_GROUP* group = aVia.GetParentGroup();

    if( !group )
        return false;

    STITCHING_GROUP_SETTINGS settings;
    return parseStitchingGroupName( group->GetName(), settings );
}


ZONE* GetZoneForViaStitchingVia( BOARD* aBoard, const PCB_VIA& aVia )
{
    if( !aBoard )
        return nullptr;

    EDA_GROUP* group = aVia.GetParentGroup();

    if( !group )
        return nullptr;

    STITCHING_GROUP_SETTINGS settings;

    if( !parseStitchingGroupName( group->GetName(), settings ) )
        return nullptr;

    for( ZONE* zone : aBoard->Zones() )
    {
        if( zone->m_Uuid.AsString() == settings.uuid )
            return zone;
    }

    return nullptr;
}


static bool RuleAreasHaveSameProps( const ZONE& a, const ZONE& b )
{
    // This function is only used to compare rule areas, so we can assume that both a and b are rule areas
    wxASSERT( a.GetIsRuleArea() && b.GetIsRuleArea() );

    return a.GetDoNotAllowZoneFills() == b.GetDoNotAllowZoneFills()
           && a.GetDoNotAllowFootprints() == b.GetDoNotAllowFootprints()
           && a.GetDoNotAllowTracks() == b.GetDoNotAllowTracks()
           && a.GetDoNotAllowVias() == b.GetDoNotAllowVias()
           && a.GetDoNotAllowPads() == b.GetDoNotAllowPads();
}


std::vector<std::unique_ptr<ZONE>> MergeZonesWithSameOutline( std::vector<std::unique_ptr<ZONE>>&& aZones )
{
    const auto polygonsAreMergeable = []( const SHAPE_POLY_SET::POLYGON& a, const SHAPE_POLY_SET::POLYGON& b ) -> bool
    {
        if( a.size() != b.size() )
            return false;

        // NOTE: this assumes the polygons have their line chains in the same order
        // But that is not actually required for same geometry (i.e. mergeability)
        for( size_t lineChainId = 0; lineChainId < a.size(); lineChainId++ )
        {
            const SHAPE_LINE_CHAIN& chainA = a[lineChainId];
            const SHAPE_LINE_CHAIN& chainB = b[lineChainId];

            // Note: this assumes the polygons are either already simplified or that it's
            // OK to not merge even if they would be the same after simplification.
            if( chainA.PointCount() != chainB.PointCount() || chainA.BBox() != chainB.BBox()
                || !chainA.CompareGeometry( chainB ) )
            {
                // Different geometry, can't merge
                return false;
            }
        }

        return true;
    };

    const auto zonesAreMergeable = [&]( const ZONE& a, const ZONE& b ) -> bool
    {
        // Can't merge rule areas with zone fills
        if( a.GetIsRuleArea() != b.GetIsRuleArea() )
            return false;

        if( a.GetIsRuleArea() )
        {
            if( !RuleAreasHaveSameProps( a, b ) )
                return false;
        }
        else
        {
            // We could also check clearances and so on
            if( a.GetNetCode() != b.GetNetCode() )
                return false;
        }

        const SHAPE_POLY_SET* polySetA = a.Outline();
        const SHAPE_POLY_SET* polySetB = b.Outline();

        if( polySetA->OutlineCount() != polySetB->OutlineCount() )
            return false;

        if( polySetA->OutlineCount() == 0 )
        {
            // both have no outline, so they are the same, but we must not
            // derefence them, as they are empty
            return true;
        }

        // REVIEW: this assumes the zones only have a single polygon in the
        const SHAPE_POLY_SET::POLYGON& polyA = polySetA->CPolygon( 0 );
        const SHAPE_POLY_SET::POLYGON& polyB = polySetB->CPolygon( 0 );

        return polygonsAreMergeable( polyA, polyB );
    };

    std::vector<std::unique_ptr<ZONE>> deduplicatedZones;

    // Map of zone indexes that we have already merged into a prior zone
    std::vector<bool> merged( aZones.size(), false );

    for( size_t i = 0; i < aZones.size(); i++ )
    {
        // This one has already been subsumed into a prior zone, so skip it
        // and it will be dropped at the end.
        if( merged[i] )
            continue;

        ZONE&                                            primary = *aZones[i];
        LSET                                             layers = primary.GetLayerSet();
        std::unordered_map<PCB_LAYER_ID, SHAPE_POLY_SET> mergedFills;

        for( size_t j = i + 1; j < aZones.size(); j++ )
        {
            // This zone has already been subsumed by a prior zone, so it
            // cannot be merged into another primary
            if( merged[j] )
                continue;

            ZONE& candidate = *aZones[j];
            bool  canMerge = zonesAreMergeable( primary, candidate );

            if( canMerge )
            {
                for( PCB_LAYER_ID layer : candidate.GetLayerSet() )
                {
                    if( SHAPE_POLY_SET* fill = candidate.GetFill( layer ) )
                        mergedFills[layer] = *fill;
                }

                layers |= candidate.GetLayerSet();
                merged[j] = true;
            }
        }

        if( layers != primary.GetLayerSet() )
        {
            for( PCB_LAYER_ID layer : primary.GetLayerSet() )
            {
                if( SHAPE_POLY_SET* fill = primary.GetFill( layer ) )
                    mergedFills[layer] = *fill;
            }

            primary.SetLayerSet( layers );

            for( const auto& [layer, fill] : mergedFills )
                primary.SetFilledPolysList( layer, fill );

            primary.SetNeedRefill( false );
            primary.SetIsFilled( true );
        }

        // Keep this zone - it's a primary (may or may not have had other zones merged into it)
        deduplicatedZones.push_back( std::move( aZones[i] ) );
    }

    return deduplicatedZones;
}


void AddZoneViaStitching( BOARD_COMMIT& aCommit, BOARD* aBoard, const ZONE& aZone )
{
    addZoneViaStitching( aCommit, aBoard, aZone, true );
}


void RemoveZoneViaStitching( BOARD_COMMIT& aCommit, BOARD* aBoard, const ZONE& aZone )
{
    if( !aBoard || aZone.GetIsRuleArea() || aZone.IsTeardropArea() || !aZone.IsOnCopperLayer()
        || aZone.GetNetCode() <= 0 )
    {
        return;
    }

    PCB_GROUP* group = findStitchingGroup( aBoard, aZone );
    std::unordered_set<PCB_VIA*> groupedVias;

    if( group )
    {
        for( EDA_ITEM* item : group->GetItems() )
        {
            if( PCB_VIA* via = dyn_cast<PCB_VIA*>( item ) )
                groupedVias.insert( via );
        }

        for( PCB_VIA* via : groupedVias )
            aCommit.Remove( via );

        aCommit.Remove( group );
    }

    if( aZone.GetViaStitchingMode() == ZONE_VIA_STITCHING_MODE::NONE )
        return;

    std::vector<VECTOR2I> points;
    collectViaStitchingPoints( aBoard, aZone, points );

    if( points.empty() )
        return;

    std::vector<PCB_VIA*> viasToRemove;

    for( PCB_TRACK* track : aBoard->Tracks() )
    {
        PCB_VIA* via = dyn_cast<PCB_VIA*>( track );

        if( !via || !isZoneStitchingCandidate( *via, aZone ) )
            continue;

        if( groupedVias.contains( via ) )
            continue;

        if( std::find( points.begin(), points.end(), via->GetPosition() ) != points.end() )
            viasToRemove.push_back( via );
    }

    for( PCB_VIA* via : viasToRemove )
        aCommit.Remove( via );
}


void RebuildZoneViaStitching( BOARD_COMMIT& aCommit, BOARD* aBoard, const ZONE& aOldZone,
                              const ZONE& aNewZone )
{
    RemoveZoneViaStitching( aCommit, aBoard, aOldZone );
    addZoneViaStitching( aCommit, aBoard, aNewZone,
                         aOldZone.GetViaStitchingMode() == ZONE_VIA_STITCHING_MODE::NONE );
}


namespace
{

struct ZONE_OVERLAP_PAIR
{
    ZONE* zoneA;
    ZONE* zoneB;
    LSET  sharedLayers;
};


struct ZONE_PRIORITY_EDGE
{
    ZONE* higher;
    ZONE* lower;
    int   countDiff;
    bool  fromArea;
};

} // namespace


static std::vector<ZONE_OVERLAP_PAIR> findOverlappingPairs( BOARD* aBoard )
{
    std::vector<ZONE_OVERLAP_PAIR> pairs;
    const ZONES&                   zones = aBoard->Zones();

    for( size_t i = 0; i < zones.size(); i++ )
    {
        ZONE* a = zones[i];

        if( a->GetIsRuleArea() || a->IsTeardropArea() || !a->IsOnCopperLayer() )
            continue;

        BOX2I bboxA = a->GetBoundingBox();

        for( size_t j = i + 1; j < zones.size(); j++ )
        {
            ZONE* b = zones[j];

            if( b->GetIsRuleArea() || b->IsTeardropArea() || !b->IsOnCopperLayer() )
                continue;

            LSET shared = a->GetLayerSet() & b->GetLayerSet();
            shared &= LSET::AllCuMask();

            if( shared.none() )
                continue;

            if( !b->GetBoundingBox().Intersects( bboxA ) )
                continue;

            bool overlaps = a->Outline()->Collide( b->Outline() )
                            || ( b->Outline()->TotalVertices() > 0
                                 && a->Outline()->Contains( b->Outline()->CVertex( 0 ) ) )
                            || ( a->Outline()->TotalVertices() > 0
                                 && b->Outline()->Contains( a->Outline()->CVertex( 0 ) ) );

            if( overlaps )
                pairs.push_back( { a, b, shared } );
        }
    }

    return pairs;
}


static std::optional<ZONE_PRIORITY_EDGE> computeConstraint( const ZONE_OVERLAP_PAIR& aPair,
                                                             BOARD* aBoard )
{
    SHAPE_POLY_SET polyA = aPair.zoneA->Outline()->CloneDropTriangulation();
    SHAPE_POLY_SET polyB = aPair.zoneB->Outline()->CloneDropTriangulation();
    polyA.ClearArcs();
    polyB.ClearArcs();

    SHAPE_POLY_SET intersection;
    intersection.BooleanIntersection( polyA, polyB );

    if( intersection.IsEmpty() )
        return std::nullopt;

    intersection.BuildBBoxCaches();

    int netCodeA = aPair.zoneA->GetNetCode();
    int netCodeB = aPair.zoneB->GetNetCode();

    // Same-net overlapping zones are cooperative, not competitive. Priority
    // between them is meaningless to the fill engine. Return no constraint
    // here; AutoAssignZonePriorities() groups them to the same priority level.
    if( netCodeA == netCodeB )
        return std::nullopt;

    int countA = 0;
    int countB = 0;

    auto countIfInOverlap = [&]( const VECTOR2I& aPos, int aNetCode, PCB_LAYER_ID aLayer )
    {
        if( !aPair.sharedLayers.test( aLayer ) )
            return;

        if( intersection.Contains( aPos ) )
        {
            if( aNetCode == netCodeA )
                countA++;
            else if( aNetCode == netCodeB )
                countB++;
        }
    };

    for( FOOTPRINT* fp : aBoard->Footprints() )
    {
        for( PAD* pad : fp->Pads() )
        {
            for( PCB_LAYER_ID layer : aPair.sharedLayers.Seq() )
            {
                if( pad->IsOnLayer( layer ) )
                {
                    countIfInOverlap( pad->GetPosition(), pad->GetNetCode(), layer );
                    break;
                }
            }
        }
    }

    for( PCB_TRACK* track : aBoard->Tracks() )
    {
        if( track->Type() != PCB_VIA_T )
            continue;

        PCB_VIA* via = static_cast<PCB_VIA*>( track );

        for( PCB_LAYER_ID layer : aPair.sharedLayers.Seq() )
        {
            if( via->IsOnLayer( layer ) )
            {
                countIfInOverlap( via->GetPosition(), via->GetNetCode(), layer );
                break;
            }
        }
    }

    if( countA == 0 && countB == 0 )
    {
        double areaA = aPair.zoneA->Outline()->Area();
        double areaB = aPair.zoneB->Outline()->Area();

        if( areaA == areaB )
            return std::nullopt;

        ZONE* higher = ( areaA < areaB ) ? aPair.zoneA : aPair.zoneB;
        ZONE* lower  = ( higher == aPair.zoneA ) ? aPair.zoneB : aPair.zoneA;
        return ZONE_PRIORITY_EDGE{ higher, lower, 0, true };
    }

    int    maxCount = std::max( countA, countB );
    int    diff = std::abs( countA - countB );
    double ratio = static_cast<double>( diff ) / maxCount;

    constexpr double SIMILARITY_THRESHOLD = 0.20;

    if( ratio < SIMILARITY_THRESHOLD )
    {
        double areaA = aPair.zoneA->Outline()->Area();
        double areaB = aPair.zoneB->Outline()->Area();

        if( areaA == areaB )
            return std::nullopt;

        ZONE* higher = ( areaA < areaB ) ? aPair.zoneA : aPair.zoneB;
        ZONE* lower  = ( higher == aPair.zoneA ) ? aPair.zoneB : aPair.zoneA;
        return ZONE_PRIORITY_EDGE{ higher, lower, diff, true };
    }

    ZONE* higher = ( countA > countB ) ? aPair.zoneA : aPair.zoneB;
    ZONE* lower  = ( higher == aPair.zoneA ) ? aPair.zoneB : aPair.zoneA;
    return ZONE_PRIORITY_EDGE{ higher, lower, diff, false };
}


static void assignPrioritiesFromGraph( const std::vector<ZONE_PRIORITY_EDGE>& aEdges,
                                       std::vector<ZONE*>&                    aAllZones )
{
    std::unordered_map<ZONE*, std::vector<ZONE*>> adj;
    std::unordered_map<ZONE*, int>                inDegree;
    std::unordered_set<ZONE*>                     inGraph;

    for( ZONE* z : aAllZones )
    {
        inDegree[z] = 0;
        inGraph.insert( z );
    }

    // Sort edges so area-based (weakest) come first, then by ascending countDiff
    std::vector<ZONE_PRIORITY_EDGE> sortedEdges = aEdges;

    std::sort( sortedEdges.begin(), sortedEdges.end(),
               []( const ZONE_PRIORITY_EDGE& a, const ZONE_PRIORITY_EDGE& b )
               {
                   if( a.fromArea != b.fromArea )
                       return a.fromArea;

                   return a.countDiff < b.countDiff;
               } );

    for( const ZONE_PRIORITY_EDGE& edge : sortedEdges )
    {
        adj[edge.higher].push_back( edge.lower );
        inDegree[edge.lower]++;
    }

    // Kahn's algorithm: sources (in-degree 0) have nothing constraining them to be lower,
    // so they are the highest-priority zones. Process them first.
    std::vector<ZONE*> queue;

    for( ZONE* z : aAllZones )
    {
        if( inDegree[z] == 0 )
            queue.push_back( z );
    }

    std::sort( queue.begin(), queue.end(),
               []( const ZONE* a, const ZONE* b )
               {
                   return a->GetAssignedPriority() < b->GetAssignedPriority();
               } );

    std::vector<ZONE*> topoOrder;
    topoOrder.reserve( aAllZones.size() );

    while( !queue.empty() )
    {
        ZONE* current = queue.front();
        queue.erase( queue.begin() );
        topoOrder.push_back( current );

        auto& neighbors = adj[current];

        std::sort( neighbors.begin(), neighbors.end(),
                   []( const ZONE* a, const ZONE* b )
                   {
                       return a->GetAssignedPriority() < b->GetAssignedPriority();
                   } );

        for( ZONE* neighbor : neighbors )
        {
            inDegree[neighbor]--;

            if( inDegree[neighbor] == 0 )
                queue.push_back( neighbor );
        }

        std::sort( queue.begin(), queue.end(),
                   []( const ZONE* a, const ZONE* b )
                   {
                       return a->GetAssignedPriority() < b->GetAssignedPriority();
                   } );
    }

    // Zones stuck in cycles get appended sorted by their current priority
    if( topoOrder.size() < aAllZones.size() )
    {
        std::unordered_set<ZONE*> ordered( topoOrder.begin(), topoOrder.end() );
        std::vector<ZONE*>        remaining;

        for( ZONE* z : aAllZones )
        {
            if( ordered.find( z ) == ordered.end() )
                remaining.push_back( z );
        }

        std::sort( remaining.begin(), remaining.end(),
                   []( const ZONE* a, const ZONE* b )
                   {
                       return a->GetAssignedPriority() < b->GetAssignedPriority();
                   } );

        for( ZONE* z : remaining )
            topoOrder.push_back( z );
    }

    // topoOrder[0] is the highest-priority zone (source node). Assign descending values.
    for( size_t i = 0; i < topoOrder.size(); i++ )
        topoOrder[i]->SetAssignedPriority( static_cast<unsigned>( topoOrder.size() - 1 - i ) );
}


static ZONE* ufFind( std::unordered_map<ZONE*, ZONE*>& aParent, ZONE* aZone )
{
    ZONE*& parent = aParent[aZone];

    if( parent != aZone )
        parent = ufFind( aParent, parent );

    return parent;
}


static void ufUnion( std::unordered_map<ZONE*, ZONE*>& aParent,
                     std::unordered_map<ZONE*, int>&    aRank,
                     ZONE* aA, ZONE* aB )
{
    ZONE* rootA = ufFind( aParent, aA );
    ZONE* rootB = ufFind( aParent, aB );

    if( rootA == rootB )
        return;

    if( aRank[rootA] < aRank[rootB] )
        std::swap( rootA, rootB );

    aParent[rootB] = rootA;

    if( aRank[rootA] == aRank[rootB] )
        aRank[rootA]++;
}


bool AutoAssignZonePriorities( BOARD* aBoard, PROGRESS_REPORTER* aReporter )
{
    std::vector<ZONE*> eligibleZones;

    for( ZONE* zone : aBoard->Zones() )
    {
        if( !zone->GetIsRuleArea() && !zone->IsTeardropArea() && zone->IsOnCopperLayer() )
            eligibleZones.push_back( zone );
    }

    if( eligibleZones.size() < 2 )
        return false;

    std::unordered_map<ZONE*, unsigned> originalPriorities;

    for( ZONE* z : eligibleZones )
        originalPriorities[z] = z->GetAssignedPriority();

    std::vector<ZONE_OVERLAP_PAIR> pairs = findOverlappingPairs( aBoard );

    if( pairs.empty() )
        return false;

    // Build equivalence classes for same-net overlapping zones. These zones
    // are cooperative and must share the same priority after assignment.
    std::unordered_map<ZONE*, ZONE*> ufParent;
    std::unordered_map<ZONE*, int>   ufRank;

    for( ZONE* z : eligibleZones )
    {
        ufParent[z] = z;
        ufRank[z] = 0;
    }

    for( const ZONE_OVERLAP_PAIR& pair : pairs )
    {
        if( pair.zoneA->GetNetCode() == pair.zoneB->GetNetCode() )
            ufUnion( ufParent, ufRank, pair.zoneA, pair.zoneB );
    }

    thread_pool&                                                tp = GetKiCadThreadPool();
    std::vector<std::future<std::optional<ZONE_PRIORITY_EDGE>>> futures;
    futures.reserve( pairs.size() );

    for( const ZONE_OVERLAP_PAIR& pair : pairs )
    {
        if( pair.zoneA->GetNetCode() == pair.zoneB->GetNetCode() )
            continue;

        futures.emplace_back( tp.submit_task(
                [&pair, aBoard]()
                {
                    return computeConstraint( pair, aBoard );
                } ) );
    }

    std::vector<ZONE_PRIORITY_EDGE> edges;

    for( auto& future : futures )
    {
        std::optional<ZONE_PRIORITY_EDGE> result = future.get();

        if( result.has_value() )
            edges.push_back( result.value() );
    }

    if( !edges.empty() )
        assignPrioritiesFromGraph( edges, eligibleZones );

    // Equalize priorities within each same-net equivalence class. Each group
    // gets the maximum priority of any member so ordering constraints from
    // different-net edges propagate to the whole group.
    std::unordered_map<ZONE*, unsigned> groupMax;

    for( ZONE* z : eligibleZones )
    {
        ZONE*    root = ufFind( ufParent, z );
        unsigned pri = z->GetAssignedPriority();
        auto&    maxPri = groupMax[root];

        if( pri > maxPri )
            maxPri = pri;
    }

    for( ZONE* z : eligibleZones )
    {
        ZONE* root = ufFind( ufParent, z );
        z->SetAssignedPriority( groupMax[root] );
    }

    for( ZONE* z : eligibleZones )
    {
        if( z->GetAssignedPriority() != originalPriorities[z] )
            return true;
    }

    return false;
}
