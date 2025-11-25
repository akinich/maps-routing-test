import streamlit as st
import googlemaps
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import folium
from streamlit_folium import st_folium

# --- PAGE CONFIG ---
st.set_page_config(page_title="Delivery Route Planner", layout="wide")

st.title("🚛 Delivery Route Planner")
st.markdown("Optimize routes for **2 vehicles** starting and ending at the same depot.")

# --- HELPER FUNCTIONS ---

def get_coordinates(addresses, gmaps):
    """Convert text addresses to (lat, lng) coordinates."""
    coords = []
    valid_addresses = []
    progress_bar = st.progress(0)
    
    for i, addr in enumerate(addresses):
        if not addr.strip(): continue
        try:
            geocode_result = gmaps.geocode(addr)
            if geocode_result:
                loc = geocode_result[0]['geometry']['location']
                coords.append((loc['lat'], loc['lng']))
                valid_addresses.append(addr)
            else:
                st.warning(f"Could not find: {addr}")
        except Exception as e:
            st.error(f"Error geocoding {addr}: {e}")
        progress_bar.progress((i + 1) / len(addresses))
        
    return valid_addresses, coords

def create_distance_matrix(coords, gmaps):
    """Ask Google for the travel distance between every pair of locations."""
    matrix = []
    origins = coords
    destinations = coords
    
    try:
        response = gmaps.distance_matrix(origins=origins, destinations=destinations, mode="driving")
        rows = response['rows']
        for row in rows:
            row_list = []
            for element in row['elements']:
                if element['status'] == 'OK':
                    row_list.append(element['distance']['value'])
                else:
                    row_list.append(9999999) 
            matrix.append(row_list)
    except Exception as e:
        st.error(f"Error in Distance Matrix: {e}")
        return None
    return matrix

def get_route_polyline(gmaps, route_coords):
    """
    Get the actual road geometry (polyline) from Google Directions API.
    route_coords: List of (lat, lng) tuples in the visited order.
    """
    if len(route_coords) < 2:
        return []

    # Origin is the first point, Destination is the last
    origin = route_coords[0]
    destination = route_coords[-1]
    
    # Waypoints are everything in between
    waypoints = route_coords[1:-1]
    
    # If there are too many waypoints, Google might split results, 
    # but for <25 stops this works in one go.
    try:
        directions_result = gmaps.directions(
            origin=origin,
            destination=destination,
            waypoints=waypoints,
            mode="driving",
            optimize_waypoints=False # IMPORTANT: Keep our optimized order!
        )
        
        if directions_result:
            # Decode the overview polyline into a list of points
            encoded_polyline = directions_result[0]['overview_polyline']['points']
            path_points = googlemaps.convert.decode_polyline(encoded_polyline)
            
            # Convert dict results {'lat': x, 'lng': y} to list of tuples [(x,y)]
            return [(p['lat'], p['lng']) for p in path_points]
            
    except Exception as e:
        st.error(f"Error fetching directions polyline: {e}")
        return []
    
    return []

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
    routing.AddDimension(
        transit_callback_index, 0, 3000000, True, dimension_name)
    distance_dimension = routing.GetDimensionOrDie(dimension_name)
    distance_dimension.SetGlobalSpanCostCoefficient(100)

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    solution = routing.SolveWithParameters(search_parameters)
    return manager, routing, solution

# --- SIDEBAR ---
with st.sidebar:
    st.header("Configuration")
    if "GOOGLE_API_KEY" in st.secrets:
        api_key = st.secrets["GOOGLE_API_KEY"]
        st.success("API Key loaded from Secrets ✅")
    else:
        api_key = st.text_input("Enter Google Maps API Key", type="password")

    default_addresses = (
        "Central Park, New York, NY\n"
        "Times Square, New York, NY\n"
        "Brooklyn Bridge, New York, NY\n"
        "Empire State Building, New York, NY\n"
        "Grand Central Terminal, New York, NY\n"
        "Statue of Liberty, New York, NY\n"
        "One World Trade Center, New York, NY\n"
        "Rockefeller Center, New York, NY"
    )
    address_input = st.text_area("List of Addresses (First = Depot)", value=default_addresses, height=200)
    num_vehicles = st.number_input("Number of Vehicles", min_value=1, max_value=4, value=2)
    run_button = st.button("Plan Routes")

# --- MAIN LOGIC ---
if "vrp_data" not in st.session_state:
    st.session_state.vrp_data = None

if run_button:
    if not api_key:
        st.error("Please provide a Google Maps API Key.")
        st.stop()

    gmaps = googlemaps.Client(key=api_key)
    raw_list = address_input.split('\n')
    
    with st.spinner("Geocoding addresses..."):
        addresses, coords = get_coordinates(raw_list, gmaps)

    if len(addresses) < 2:
        st.error("Need at least 2 valid addresses.")
        st.stop()

    with st.spinner("Calculating distance matrix..."):
        dist_matrix = create_distance_matrix(coords, gmaps)

    if dist_matrix:
        with st.spinner("Optimizing routes..."):
            manager, routing, solution = solve_vrp(dist_matrix, num_vehicles, depot_index=0)
            
            if solution:
                st.session_state.vrp_data = {
                    "addresses": addresses,
                    "coords": coords,
                    "manager": manager,
                    "routing": routing,
                    "solution": solution,
                    "num_vehicles": num_vehicles
                }
            else:
                st.warning("No solution found.")
    else:
        st.error("Failed to generate matrix.")

# --- DISPLAY LOGIC ---
if st.session_state.vrp_data:
    data = st.session_state.vrp_data
    coords = data["coords"]
    addresses = data["addresses"]
    solution = data["solution"]
    routing = data["routing"]
    manager = data["manager"]
    num_vehicles = data["num_vehicles"]

    # Re-initialize gmaps for the display phase
    if not api_key: api_key = st.secrets.get("GOOGLE_API_KEY", "")
    gmaps = googlemaps.Client(key=api_key)

    st.success("Routes optimized successfully!")
    
    # Initialize Map
    m = folium.Map(location=coords[0], zoom_start=12)
    colors = ['red', 'blue', 'green', 'purple', 'orange']
    
    # Plot markers (Stops)
    for i, (lat, lng) in enumerate(coords):
        color = "black" if i == 0 else "gray"
        icon = "home" if i == 0 else "info-sign"
        folium.Marker(
            [lat, lng], 
            popup=f"{i}: {addresses[i]}",
            icon=folium.Icon(color=color, icon=icon)
        ).add_to(m)

    col1, col2 = st.columns([2, 1])
    
    with col1:
        st.subheader("Map Visualization")
        
        # DRAW ROUTES
        for vehicle_id in range(num_vehicles):
            index = routing.Start(vehicle_id)
            vehicle_route_coords = [] # Stores (lat, lng) for this vehicle's stops
            
            # Extract the ordered stops from the solver
            while not routing.IsEnd(index):
                node_index = manager.IndexToNode(index)
                vehicle_route_coords.append(coords[node_index])
                index = solution.Value(routing.NextVar(index))
            
            # Add Depot (end)
            node_index = manager.IndexToNode(index)
            vehicle_route_coords.append(coords[node_index])
            
            # Get Actual Road Path from Google Directions API
            route_color = colors[vehicle_id % len(colors)]
            
            # Call the new function to get detailed road geometry
            detailed_path = get_route_polyline(gmaps, vehicle_route_coords)
            
            if detailed_path:
                folium.PolyLine(
                    detailed_path, 
                    color=route_color, 
                    weight=5, 
                    opacity=0.8,
                    tooltip=f"Vehicle {vehicle_id + 1}"
                ).add_to(m)
            else:
                # Fallback to straight lines if Directions API fails or isn't enabled
                folium.PolyLine(
                    vehicle_route_coords, 
                    color=route_color, 
                    weight=5, 
                    dash_array='5, 10',
                    opacity=0.5
                ).add_to(m)
        
        st_folium(m, width=700, height=500)

    with col2:
        st.subheader("Route Details")
        for vehicle_id in range(num_vehicles):
            st.markdown(f"**🚗 Vehicle {vehicle_id + 1}**")
            index = routing.Start(vehicle_id)
            route_distance = 0
            route_text = ""
            
            while not routing.IsEnd(index):
                node_index = manager.IndexToNode(index)
                route_text += f"{addresses[node_index]} ➡️ <br>"
                previous_index = index
                index = solution.Value(routing.NextVar(index))
                route_distance += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)
            
            node_index = manager.IndexToNode(index)
            route_text += f"🏁 Return to {addresses[node_index]}"
            
            st.markdown(f"📏 Distance: {route_distance / 1000:.2f} km")
            st.caption(route_text, unsafe_allow_html=True)
            st.divider()
