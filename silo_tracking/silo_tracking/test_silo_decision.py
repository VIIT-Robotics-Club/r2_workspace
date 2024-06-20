def silo_decision(balls_in_silos, our_ball_color, opponent_ball_color):
    # Function to prioritize the silos based on the conditions
    def evaluate_silo(silo, balls):
        print("Silo: ", silo)
        print("Balls: ", balls)
        
        our_ball_count = balls.count(our_ball_color)
        print("Our ball count: ", our_ball_count)
        opponent_ball_count = balls.count(opponent_ball_color)
        print("Opponent ball count: ", opponent_ball_count)
        is_full = len(balls) == 3
        print("Is full: ", is_full)
        top_is_our_ball = balls[-1] == our_ball_color if balls else False
        print("Top is our ball: ", top_is_our_ball)
        
        #### CONDITION WHEN SILO IS FULL ####
        if is_full:
            return 10
        
        ### CONDITIONS WHEN 2 BALLS ARE IN THE SILO ###
        if silo_pts_diff <= 0:
            # Silo pt for Opponent - opponent's balls
            if opponent_ball_count == 2 and our_ball_count == 0:
                return 0
            
            # Silo pt for anyone: opponent added the last ball
            if opponent_ball_count == 1 and our_ball_count == 1 and not top_is_our_ball:
                return 1
            
            # Silo pt for anyone: we added the last ball
            if opponent_ball_count == 1 and our_ball_count == 1 and top_is_our_ball:
                return 2
            
            # Silo pt for us
            if our_ball_count == 2 and opponent_ball_count == 0:
                return 3
        else: 
            # Silo pt for Opponent - opponent's balls
            if opponent_ball_count == 2 and our_ball_count == 0:
                return 3
            
            # Silo pt for anyone: opponent added the last ball
            if opponent_ball_count == 1 and our_ball_count == 1 and not top_is_our_ball:
                return 0
            
            # Silo pt for anyone: we added the last ball
            if opponent_ball_count == 1 and our_ball_count == 1 and top_is_our_ball:
                return 1
            
            # Silo pt for us
            if our_ball_count == 2 and opponent_ball_count == 0:
                return 2
        
        ### CONDITION WHEN NO BALLS ARE IN THE SILO ###
        if (our_ball_count + opponent_ball_count) == 0:
            return 4
        
        ### ONE BALL CONDITIONS ###
        # Put another ball to ensure opponent doesn't get silo point
        if our_ball_count == 1:
            return 5
        
        # Low priority, as opponent can get silo point if we put another ball
        if opponent_ball_count == 1:
            return 6
                    
        return 7  # Lowest priority

    # Calculate difference in silo points
    our_points = 0
    opponent_points = 0
    
    for balls in balls_in_silos.values():
        if len(balls) >= 2 and balls[-1] == our_ball_color and balls.count(our_ball_color) >= 2:
            our_points += 1
        elif len(balls) >= 2 and balls[-1] == opponent_ball_color and balls.count(opponent_ball_color) >= 2:
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
    
# Testing
balls_in_silos = {'s1': ['b', 'b', ], 's2': ['r',], 's3': ['r','b','b'], 's4': ['b','b','b'], 's5': ['b',]}
our_ball_color = 'b'
opponent_ball_color = 'r'
result = silo_decision(balls_in_silos, our_ball_color, opponent_ball_color)
print(result)
