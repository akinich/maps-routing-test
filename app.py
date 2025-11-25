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
    
    # Simple progress bar
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
                    row_list.append(element['distance']['value']) # Value in meters
                else:
                    row_list.append(9999999) 
            matrix.append(row_list)
            
    except Exception as e:
        st.error(f"Error in Distance Matrix: {e}")
        return None
        
    return matrix

def solve_vrp(distance_matrix, num_vehicles, depot_index=0):
    """Uses Google OR-Tools to solve the routing problem."""
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
        transit_callback_index,
        0, 
        3000000, 
        True, 
        dimension_name)
    distance_dimension = routing.GetDimensionOrDie(dimension_name)
    distance_dimension.SetGlobalSpanCostCoefficient(100)

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    solution = routing.SolveWithParameters(search_parameters)
    return manager, routing, solution

# --- SIDEBAR & INPUTS ---
with st.sidebar:
    st.header("Configuration")
    
    # Secrets handling
    if "GOOGLE_API_KEY" in st.secrets:
        api_key = st.secrets["GOOGLE_API_KEY"]
        st.success("API Key loaded from Secrets ✅")
    else:
        api_key = st.text_input("Enter Google Maps API Key", type="password")
    
    default_addresses = (
        "Central Park, New York, NY\n"
        "Times Square, New
