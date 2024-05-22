(define (problem go-to) 
  (:domain example)
  (:objects
    bot - robot
    room1 room2 room3 room4 - room
  )
  (:init
    (in bot room1) ;; Starting room should be dynamically set if needed
    ;; Connections between rooms based on the layout in the image
    (cangonorth room1 room4)
    (cangonorth room2 room3)
    (cangoeast room1 room3)
    (cangoeast room2 room4)
    (cangosouth room4 room1)
    (cangosouth room3 room2)
    (cangowest room3 room1)
    (cangowest room4 room2)
  )
  (:goal
    (in bot room2)
  )
)