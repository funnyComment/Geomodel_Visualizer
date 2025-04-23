import streamlit as st
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd
import numpy as np
import copy
import os

from utils import load_geomodel, compute_robot_pose, draw_robot

st.set_page_config(layout="wide")
st.title("🤖 Robot DH Editor")

# Initialize session state
if "robot_data" not in st.session_state:
    st.session_state.robot_data = {}
    st.session_state.robot_data_original = {}
    st.session_state.last_selected_link = None

# File uploader
uploaded_file = st.sidebar.file_uploader("Choose a .geo file", type=["geo"])

if uploaded_file:
    # Save to temp and load
    temp_path = os.path.join("temp.geo")
    with open(temp_path, "wb") as f:
        f.write(uploaded_file.read())

    robot_data = load_geomodel(temp_path)
    if robot_data:
        st.session_state.robot_data = robot_data
        st.session_state.robot_data_original = copy.deepcopy(robot_data)
        st.session_state.last_selected_link = None

# Ensure data is loaded
if not st.session_state.robot_data:
    st.info("📁 Upload a .geo file to begin.")
    st.stop()

robot_data = st.session_state.robot_data

# Reset button FIRST (to rerun before widgets are rendered)
if st.sidebar.button("🔁 Reset model"):
    st.session_state.robot_data = copy.deepcopy(st.session_state.robot_data_original)
    for link_name, params in st.session_state.robot_data["Links"].items():
        for k in ["A", "r", "d", "T"]:
            state_key = f"{link_name}_{k}"
            st.session_state[state_key] = params[k]
    st.session_state.last_selected_link = None
    st.experimental_rerun()

# Link selector
link_names = list(robot_data["Links"].keys())
selected_link = st.sidebar.selectbox("Select Link", link_names)

# Track selected link to update sliders
if st.session_state.last_selected_link != selected_link:
    st.session_state.last_selected_link = selected_link

# Read current DH values from session state or initialize
current_params = robot_data["Links"][selected_link]
default_A = st.session_state.get(f"{selected_link}_A", current_params["A"])
default_r = st.session_state.get(f"{selected_link}_r", current_params["r"])
default_d = st.session_state.get(f"{selected_link}_d", current_params["d"])
default_T = st.session_state.get(f"{selected_link}_T", current_params["T"])

# Sliders for DH parameters
A = st.sidebar.slider("A (alpha)", -3.14, 3.14, default_A)
r = st.sidebar.slider("r (a)", -2.0, 2.0, default_r)
d = st.sidebar.slider("d", -2.0, 2.0, default_d)
T = st.sidebar.slider("T (theta)", -3.14, 3.14, default_T)

# Update session state and robot data
st.session_state[f"{selected_link}_A"] = A
st.session_state[f"{selected_link}_r"] = r
st.session_state[f"{selected_link}_d"] = d
st.session_state[f"{selected_link}_T"] = T
robot_data["Links"][selected_link] = {"A": A, "r": r, "d": d, "T": T}

# Compute DH parameters and poses
dh_params = [(link["A"], link["r"], link["d"], link["T"]) for link in robot_data["Links"].values()]
poses = compute_robot_pose(dh_params)

# Plot robot
st.subheader("📡 Robot Visualization")
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
draw_robot(poses, ax=ax)
plt.tight_layout()
st.pyplot(fig)

# Display parameters table
st.subheader("📋 DH Parameters Table (A, r, d, T)")
df = pd.DataFrame(robot_data["Links"]).T
df = df[["A", "r", "d", "T"]]
st.dataframe(df.round(4))
