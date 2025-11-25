import streamlit as st
import googlemaps
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import folium
from streamlit_folium import st_folium
import math

# --- PAGE CONFIG ---
st.set_page_config(page_title="Delivery Route Planner", layout="wide")

st.title("🚛 Delivery Route Planner")
st.markdown("Build your route by adding stops below.")

# --- SESSION STATE ---
if "stops" not in st.session_state:
    st.session_state.stops = []
if "depot" not in st.session_state:
    st.session_state.depot = None
if "vrp_data" not in st.session_state:
    st.session_state.vrp_data = None

# --- GEOMETRY & MATH HELPERS (NEW) ---

def haversine(coord1, coord2):
    """Calculate distance in meters between two lat/lng tuples."""
    R = 6371000  # Radius of Earth in meters
    lat1, lon1 = math.radians(coord1[0]), math.radians(coord1[1])
    lat2, lon2 = math.radians(coord2[0]), math.radians(coord2[1])
    
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    
    a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    
    return R * c

def calculate_bearing(coord1, coord2):
    """Calculate the compass bearing between two points."""
    lat1, lon1 = math.radians(coord1[0]), math.radians(coord1[1])
    lat2, lon2 = math.radians(coord2[0]), math.radians(coord2[1])
    
    dlon = lon2 - lon1
    
    x = math.sin(dlon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1) * math.cos(lat2) * math.cos(dlon))
    
    initial_bearing = math.atan2(x, y)
    initial_bearing = math.degrees(initial_bearing)
    compass_bearing = (initial_bearing + 360) % 360
    return compass_bearing

def get_interval_arrows(path, interval_meters=3000):
    """
    Walks along the path and returns a list of (lat, lng, bearing) 
    tuples exactly every `interval_meters`.
    """
    arrows = []
    dist_accum = 0
    
    for i in range(len(path) - 1):
        p1 = path[i]
        p2 = path[i+1]
        
        # Distance of this tiny segment
        seg_dist = haversine(p1, p2)
        
        # If adding this segment passes the threshold
        if dist_accum + seg_dist >= interval_meters:
            # How far into this segment is the 3km mark?
            remaining = interval_meters - dist_accum
            ratio = remaining / seg_dist
            
            # Interpolate the exact point
            new_lat = p1[0] + (p2[0] - p1[0]) * ratio
            new_lng = p1[1] + (p2[1] - p1[1]) * ratio
            
            # Calculate bearing for arrow direction
            bearing = calculate_bearing(p1, p2)
            
            arrows.append((new_lat, new_lng, bearing))
            
            # Reset accumulator, effectively starting "fresh" from the new point
            # (Simplification: we treat the remainder of this segment as start of next interval)
            dist_accum = seg_dist - remaining
        else:
            dist_accum += seg_dist
            
    return arrows

# --- GOOGLE MAPS HELPERS ---

def validate_and_get_geocode(address, gmaps):
    if not address: return None, None
    try:
        geocode_result = gmaps.geocode(address)
        if geocode_result:
            formatted_addr = geocode_result[0]['formatted_address']
            loc = geocode_result[0]['geometry']['location']
            return formatted_addr, (loc['lat'], loc['lng'])
    except Exception as e:
        st.error(f"API Error: {e}")
    return None, None

def get_route_polyline(gmaps, route_coords):
    if len(route_coords) < 2: return []
    try:
        directions_result = gmaps.directions(
            origin=route_coords[0],
            destination=route_coords[-1],
            waypoints=route_coords[1:-1],
            mode="driving",
            optimize_waypoints=False
        )
        if directions_result:
            encoded_polyline = directions_result[0]['overview_polyline']['points']
            path_points = googlemaps.convert.decode_polyline(encoded_polyline)
            return [(p['lat'], p['lng']) for p in path_points]
    except:
        return []

def create_distance_matrix(coords, gmaps):
    matrix = []
    try:
        response = gmaps.distance_matrix(origins=coords, destinations=coords, mode="driving")
        for row in response['rows']:
            row_list = []
            for element in row['elements']:
                val = element['distance']['value'] if element['status'] == 'OK' else 9999999
                row_list.append(val)
            matrix.append(row_list)
    except Exception as e:
        st.error(f"Distance Matrix Error: {e}")
        return None
    return matrix

def solve_vrp(distance_matrix, num_vehicles, depot_index=0):
    manager = pywrapcp.RoutingIndexManager(len(distance_matrix), num_vehicles, depot_index)
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    dimension_name = 'Distance'
    routing.AddDimension(transit_callback_index, 0, 3000000, True, dimension_name)
    distance_dimension = routing.GetDimensionOrDie(dimension_name)
    distance_dimension.SetGlobalSpanCostCoefficient(100)

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    
    solution = routing.SolveWithParameters(search_parameters)
    return manager, routing, solution

# --- SIDEBAR ---
with st.sidebar:
    st.header("⚙️ Configuration")
    if "GOOGLE_API_KEY" in st.secrets:
        api_key = st.secrets["GOOGLE_API_KEY"]
    else:
        api_key = st.text_input("Google Maps API Key", type="password")

    if not api_key:
        st.warning("Enter API Key to start.")
        st.stop()
        
    gmaps = googlemaps.Client(key=api_key)

    st.divider()
    
    # DEPOT
    st.subheader("1. Start Point (Depot)")
    if st.session_state.depot is None:
        depot_input = st.text_input("Search for Depot address", placeholder="e.g. Times Square")
        if st.button("Set Depot") and depot_input:
            formatted, coords = validate_and_get_geocode(depot_input, gmaps)
            if formatted:
                st.session_state.depot = {'address': formatted, 'coords': coords}
                st.rerun()
            else:
                st.error("Address not found.")
    else:
        st.success(f"📍 **Depot:** {st.session_state.depot['address']}")
        if st.button("Change Depot"):
            st.session_state.depot = None
            st.rerun()

    st.divider()

    # STOPS
    st.subheader("2. Add Delivery Stops")
    with st.form("add_stop_form", clear_on_submit=True):
        new_stop_input = st.text_input("Search for Stop", placeholder="e.g. Empire State Building")
        add_submitted = st.form_submit_button("Add Stop")
    
    if add_submitted and new_stop_input:
        formatted, coords = validate_and_get_geocode(new_stop_input, gmaps)
        if formatted:
            st.session_state.stops.append({'address': formatted, 'coords': coords})
            st.toast(f"Added: {formatted}", icon="✅")
        else:
            st.error("Could not find that location.")

    if st.session_state.stops:
        st.markdown("### Current List")
        for i, stop in enumerate(st.session_state.stops):
            col1, col2 = st.columns([4, 1])
            with col1:
                st.text(f"{i+1}. {stop['address'][:30]}...")
            with col2:
                if st.button("❌", key=f"del_{i}"):
                    st.session_state.stops.pop(i)
                    st.rerun()
        
        if st.button("Clear All Stops"):
            st.session_state.stops = []
            st.rerun()
            
    st.divider()
    num_vehicles = st.number_input("Vehicles", min_value=1, max_value=4, value=2)
    plan_button = st.button("Plan Route 🚀", type="primary")


# --- MAIN LOGIC ---

if plan_button:
    if not st.session_state.depot:
        st.error("Please set a Depot first.")
    elif not st.session_state.stops:
        st.error("Please add at least one delivery stop.")
    else:
        all_stops = [st.session_state.depot] + st.session_state.stops
        coords_list = [s['coords'] for s in all_stops]
        address_list = [s['address'] for s in all_stops]

        with st.spinner("Calculating optimal routes..."):
            dist_matrix = create_distance_matrix(coords_list, gmaps)
            if dist_matrix:
                manager, routing, solution = solve_vrp(dist_matrix, num_vehicles, depot_index=0)
                if solution:
                    st.session_state.vrp_data = {
                        "addresses": address_list,
                        "coords": coords_list,
                        "manager": manager,
                        "routing": routing,
                        "solution": solution,
                        "num_vehicles": num_vehicles
                    }
                else:
                    st.error("No solution found.")

# --- VISUALIZATION ---

if st.session_state.vrp_data:
    data = st.session_state.vrp_data
    coords = data["coords"]
    addresses = data["addresses"]
    solution = data["solution"]
    routing = data["routing"]
    manager = data["manager"]
    num_vehicles = data["num_vehicles"]
    
    gmaps = googlemaps.Client(key=api_key)

    st.success(f"Routes planned for {len(addresses)-1} stops!")

    m = folium.Map(location=coords[0], zoom_start=13, tiles="CartoDB positron")
    
    folium.TileLayer(
        tiles = 'https://mt1.google.com/vt/lyrs=y&x={x}&y={y}&z={z}',
        attr = 'Google',
        name = 'Google Satellite',
        overlay = False,
        control = True
    ).add_to(m)

    # Plot Stops
    for i, (lat, lng) in enumerate(coords):
        color = "black" if i == 0 else "blue"
        icon = "home" if i == 0 else "info-sign"
        popup_text = "DEPOT" if i == 0 else f"Stop {i}"
        
        folium.Marker(
            [lat, lng], 
            popup=f"{popup_text}: {addresses[i]}",
            icon=folium.Icon(color=color, icon=icon)
        ).add_to(m)

    col1, col2 = st.columns([2, 1])
    
    with col1:
        st.subheader("🗺️ Route Map")
        
        for vehicle_id in range(num_vehicles):
            index = routing.Start(vehicle_id)
            vehicle_route_coords = []
            
            while not routing.IsEnd(index):
                node_index = manager.IndexToNode(index)
                vehicle_route_coords.append(coords[node_index])
                index = solution.Value(routing.NextVar(index))
            
            node_index = manager.IndexToNode(index)
            vehicle_route_coords.append(coords[node_index])
            
            route_color = ['red', 'blue', 'green', 'purple'][vehicle_id % 4]
            detailed_path = get_route_polyline(gmaps, vehicle_route_coords)
            path_to_draw = detailed_path if detailed_path else vehicle_route_coords
            
            # 1. Draw the Route Line (Clean, no text)
            folium.PolyLine(
                path_to_draw, 
                color=route_color, 
                weight=6, 
                opacity=0.6, 
                tooltip=f"Vehicle {vehicle_id+1}"
            ).add_to(m)
            
            # 2. Calculate and Draw Arrows every 3km
            # We use 3000 meters as the interval
            arrow_points = get_interval_arrows(path_to_draw, interval_meters=3000)
            
            for (lat, lng, bearing) in arrow_points:
                folium.RegularPolygonMarker(
                    location=[lat, lng],
                    fill_color=route_color,
                    color=route_color,
                    number_of_sides=3,   # Triangle
                    radius=6,            # Size
                    rotation=bearing,    # Rotate to match road direction
                    fill_opacity=1.0,
                    opacity=1.0
                ).add_to(m)

        folium.LayerControl().add_to(m)
        st_folium(m, width=700, height=500)

    with col2:
        st.subheader("📋 Drivers' Manifest")
        for vehicle_id in range(num_vehicles):
            st.markdown(f"**🚗 Vehicle {vehicle_id + 1}**")
            index = routing.Start(vehicle_id)
            route_distance = 0
            steps = []
            
            while not routing.IsEnd(index):
                node_index = manager.IndexToNode(index)
                addr = addresses[node_index]
                if node_index == 0:
                    steps.append(f"🏁 **Start**: {addr}")
                else:
                    steps.append(f"📦 **Drop**: {addr}")
                
                previous_index = index
                index = solution.Value(routing.NextVar(index))
                route_distance += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)
            
            node_index = manager.IndexToNode(index)
            steps.append(f"🛑 **End**: {addresses[node_index]}")
            
            st.info(f"Total Distance: {route_distance / 1000:.1f} km")
            for step in steps:
                st.write(step)
            st.divider()

elif st.session_state.depot is None:
    st.info("👈 Please start by setting your Depot address in the sidebar.")
