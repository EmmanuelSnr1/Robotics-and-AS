import os
from pyperplan.pddl.parser import Parser
from pyperplan.grounding import ground
from pyperplan.search import a_star
from pyperplan.heuristics.relaxation import hFFHeuristic  # Import hFF heuristic
from pyperplan.search.searchspace import make_root_node

class PDDLPlanner:
    def __init__(self, domain_file, problem_file):
        self.domain_file = domain_file
        self.problem_file = problem_file

    def plan(self):
        # Parse the domain and problem files
        domain_path = os.path.join(os.getcwd(), self.domain_file)
        problem_path = os.path.join(os.getcwd(), self.problem_file)
        parser = Parser(domain_path, problem_path)
        domain = parser.parse_domain()
        problem = parser.parse_problem(domain)
        
        # Ground the problem
        print("Grounding the problem...")
        task = ground(problem)
        print(f"Grounded Task: {task}")
        
        # Use the hFF heuristic
        heuristic = hFFHeuristic(task)

        # Create the initial search node
        print("Creating initial search node...")
        root_node = make_root_node(task.initial_state)
        print(f"Initial State: {root_node.state}")

        # Use A* search algorithm to find a plan
        print("Searching for a plan...")
        solution = a_star.astar_search(task, heuristic)
        
        if solution:
            plan = [str(action) for action in solution]
            return plan
        else:
            print("No plan found.")
            return []

# Example usage:
if __name__ == "__main__":
    planner = PDDLPlanner('domain.pddl', 'problem.pddl')
    plan = planner.plan()
    for step in plan:
        print(step)
