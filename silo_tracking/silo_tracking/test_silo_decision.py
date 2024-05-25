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
            
            
            #### CONDITION WHEN SILO IS FULL ####
            if (is_full==True):
                return 10
            
            
            ###CONDITIONS WHEN 2 BALLS ARE IN THE SILO ###
            
            if silo_pts_diff <= 0:
                # Silo pt for Opponent - RR
                if red_count == 2 and blue_count == 0:
                    return 0
                
                # Silo pt for anyone: opponent added the last ball(red) -- BR
                if red_count==1 and blue_count==1 and top_is_blue==False:
                    return 1
                
                # Silo pt for anyone: we added the last ball(blue) -- RB
                if (red_count==1 and blue_count==1 and top_is_blue==True):
                    return 2
                
                #Silo pt for us -- BB
                if blue_count==2 and red_count==0:
                    return 3
            
            else: 
                # Silo pt for Opponent - RR
                if red_count == 2 and blue_count == 0:
                    return 3
                
                # Silo pt for anyone: opponent added the last ball(red) -- BR
                if red_count==1 and blue_count==1 and top_is_blue==False:
                    return 0
                
                # Silo pt for anyone: we added the last ball(blue) -- RB
                if (red_count==1 and blue_count==1 and top_is_blue==True):
                    return 1
                
                #Silo pt for us -- BB
                if blue_count==2 and red_count==0:
                    return 2
            
            
            ### CONDITION WHEN NO BALLS ARE IN THE SILO ###
            if ((blue_count+red_count)==0):
                return 4
            
            ### ONE BALL CONDITIONS ###
        
            # Put another blue to ensure opponent doesn't get silo point
            if (blue_count == 1):
                return 5
            
            # Low priority, as opponent can get silo point if we put another blue ball
            if (red_count == 1):
                return 6
                        
            return 7  # Lowest priority
        
        # Calculate difference in silo pts
        our_points = 0
        opponent_points = 0
        
        for balls in balls_in_silos.values():
            if len(balls) >= 2 and balls[-1] == 'b' and balls.count('b') >= 2:
                our_points += 1
            elif len(balls) >= 2 and balls[-1] == 'r' and balls.count('r') >= 2:
                opponent_points += 1
        
        silo_pts_diff = our_points - opponent_points
        print("Our Points: ", our_points)
        print("Opponent Points: ", opponent_points)
        print("Silo Points Difference: ", silo_pts_diff)

        silo_scores = {silo: evaluate_silo(silo, balls) for silo, balls in balls_in_silos.items()}
        
        # Sort silos by their score and then by their number if scores are equal
        sorted_silos = sorted(silo_scores.keys(), key=lambda s: (silo_scores[s], s))
              
        print("Silo scores: ", silo_scores)
        print("Sorted silos: ", sorted_silos)

        # Return the silo with the highest priority (lowest score)
        return sorted_silos[0] if sorted_silos else None
    
    
    
balls_in_silos = {'s1': ['b', 'r', 'r'], 's2': ['b','r','b'], 's3': ['r','b','b'], 's4': ['b','b','b'], 's5': []}

# balls_in_silos2 = {'s1': ['b', 'r', 'r'], 's2': ['b'], 's3': ['r'], 's4': ['b'], 's5': ['b','r']}
result = silo_decision()
print(result)
    
