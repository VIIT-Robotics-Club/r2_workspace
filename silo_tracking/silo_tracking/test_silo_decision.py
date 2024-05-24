def silo_decision():
        # Function to prioritize the silos based on the conditions
        def evaluate_silo(silo, balls):
            print("Silo: ", silo)
            print("Balls: ", balls)
            
            blue_count = balls.count('b')
            print("Blue count: ", blue_count)
            red_count = balls.count('r')
            print("Red count: ", red_count)
            is_full = len(balls) == 3
            print("Is full: ", is_full)
            top_is_blue = balls[-1] == 'b' if balls else False
            print("Top is blue: ", top_is_blue)

            if (is_full==True):
                return 10
            
            if (is_full==False) and len(balls) == 2 :
                return 0  # Highest priority

            if (blue_count ==0 and red_count == 1):            
                return 1
            # For individual points, prioritize silos with fewer balls
            # Prefer silos that already have blue balls
            if blue_count > 0:
                return 1  # Next highest priority

            return 2  # Lowest priority

        silo_scores = {silo: evaluate_silo(silo, balls) for silo, balls in balls_in_silos.items()}
        
        # Sort silos by their score and then by their number if scores are equal
        sorted_silos = sorted(silo_scores.keys(), key=lambda s: (silo_scores[s], s))
              
        print("Silo scores: ", silo_scores)
        print("Sorted silos: ", sorted_silos)

        # Return the silo with the highest priority (lowest score)
        return sorted_silos[0] if sorted_silos else None
    
    
balls_in_silos = {'s1': ['b', 'r', 'r'], 's2': ['b'], 's3': ['r'], 's4': [], 's5': []}

result = silo_decision()
print(result)
    
