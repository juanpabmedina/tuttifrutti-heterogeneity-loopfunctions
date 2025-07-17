import xml.etree.ElementTree as ET
import xml.dom.minidom
import os 
import subprocess
import stat
import ast
import pandas as pd 
import numpy as np 
def modify_fsm_config(xml_file_path, controller_id, new_fsm_config):
    # Parse the XML file
    tree = ET.parse(xml_file_path)
    root = tree.getroot()

    # Find the specified automode_controller by id
    automode_controllers = root.findall('.//automode_controller[@id="{}"]'.format(controller_id))
    for automode_controller in automode_controllers:
        # Find the params element inside the automode_controller
        params_element = automode_controller.find('./params')
        if params_element is not None:
            # Update the fsm-config attribute
            params_element.set('fsm-config', new_fsm_config)

    # Save the modified XML back to the file
    tree.write(xml_file_path)

def modify_quantity(xml_file_path, distribute_id, new_quantity, total_robots):
    # Parse the XML file
    tree = ET.parse(xml_file_path)
    root = tree.getroot()
    # Find the specified e-puck by id
    distribute_elements = root.findall('.//distribute[@id="{}"]'.format(distribute_id))
    for distribute_element in distribute_elements:
        # Find the parent entity element
        entity_element = distribute_element.find('./entity')
        if entity_element is not None:
            # Update the quantity attribute
            entity_element.set('quantity', str(new_quantity))

    entity_element = root.find('.//loop_functions/params')
    if entity_element is not None:
            # Update the quantity attribute
            entity_element.set('number_robots', str(total_robots))

    # Save the modified XML back to the file
    tree.write(xml_file_path)


def run_Argos(argos):
    # run argos with passed fsm to write hitory file
    with open("./argos.sh",'w+') as f:
        f.write("#!/usr/bin/env python\n")
        f.write(f"argos3 -c {argos}")

    st = os.stat('./argos.sh')
    os.chmod('./argos.sh', st.st_mode | stat.S_IEXEC)

    subprocess.run(["bash", 'argos.sh'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

# Example usage
argos_file_path = 'chain_robots.argos'

p1 = 0.9
p2 = 0.5
p3 = 0.1

p = 0.9 

#f'--nstates 2 --s0 8 --vel0 1 --cle0 0 --clr0 3 --n0 2 --n0x0 0 --c0x0 0 --p0x0 {p1} --n0x1 0 --c0x1 2 --p0x1 {p1} --s1 9 --vel1 1 --cle1 0 --clr1 2 --n1 2 --n1x0 0 --c1x0 2 --p1x0 {p1} --n1x1 0 --c1x1 0 --p1x1 {p1}',

####### Chain of robots #######
fsm_config_dict = {
    1: '--nstates 1 --s0 3 --fov0 1 --vel0 1.0 --clr0 2 --phe0 1',
    2: '--nstates 1 --s0 3 --fov0 1 --vel0 1.0 --clr0 1 --phe0 1',
    3: '--nstates 1 --s0 2 --fov0 1 --vel0 1.0 --clr0 3 --phe0 1',
}
new_quantity = [25,15,10]

####### Prueba tandem recruiting #######

# fsm_config_dict = {
#     1: '--nstates 1 --s0 0 --rwm0 5 --phe0 1',
#     2: '--nstates 1 --s0 2 --fov0 1 --vel0 1.0 --clr0 1 --phe0 1',
#     3: '--nstates 1 --s0 2 --fov0 1 --vel0 1.0 --clr0 3 --phe0 1',
#     4: '--nstates 1 --s0 2 --fov0 1 --vel0 1.0 --clr0 4 --phe0 1'
# }

# new_quantity = 5
# new_quantity = [1,1,0,0]


total_robots = new_quantity*4

for n in range(1,len(new_quantity)+1):
    controller_id = 'heterogeneity'+str(n)
    new_fsm_config = fsm_config_dict[n]
    distribute_id = 'swarm'+str(n)
    modify_fsm_config(argos_file_path, controller_id, new_fsm_config)
    modify_quantity(argos_file_path, distribute_id, new_quantity[n-1], total_robots)

n_exp = 1

# os.remove('./score.txt')

for _ in range(n_exp):
    run_Argos(argos_file_path)

# score = open(f'/home/robotmaster/heterogeneous_swarms/score.txt')
# data_score = []

# with open(f'/home/robotmaster/heterogeneous_swarms/score.txt', 'r') as f:
#     for score in f.readlines(): 
#         data_score.append(ast.literal_eval(score[:-1]))

# print(data_score)
# print(np.mean(data_score))

# exp_name = 'ForagingTutti_rwm_rep'

# # Example data to append
# new_data = {'name': exp_name, 'fsm': fsm_config_dict, 'swarm': new_quantity, 'score': data_score}

# # Path to the CSV file
# csv_file_path = '/home/robotmaster/heterogeneous_swarms/score.csv'

# # Try to read the existing CSV file, or create a new DataFrame if the file doesn't exist
# try:
#     df = pd.read_csv(csv_file_path)
# except FileNotFoundError:
#     # If the file doesn't exist, create a new DataFrame with the specified columns
#     df = pd.DataFrame(columns=['name', 'fsm', 'swarm', 'score'])

# # Create a new DataFrame from the new_data
# new_df = pd.DataFrame([new_data])

# # Concatenate the existing DataFrame and the new DataFrame along the rows (axis=0)
# df = pd.concat([df, new_df], ignore_index=True)

# # Write the updated DataFrame back to the CSV file
# df.to_csv(csv_file_path, index=False)