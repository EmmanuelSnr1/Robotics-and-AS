(define (problem warehouse-problem)
  (:domain warehouse)
  (:objects
    r1 - robot
    l1 l2 l3 l4 - location
    red green ducks balls - item
  )
  (:init
    (at r1 l1)
    (item_at red l2)
    (item_at green l3)
    (item_at ducks l4)
    (item_at balls l1)
    (connected l1 l2)
    (connected l2 l3)
    (connected l3 l4)
    (connected l4 l1)
  )
  (:goal
    (and (item_at red l1) (item_at green l1) (item_at ducks l1) (item_at balls l1))
  )
)
