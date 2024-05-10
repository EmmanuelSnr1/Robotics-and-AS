(define (domain warehouse)
  (:requirements :strips :typing)
  (:types robot location item)
  (:predicates
    (at ?r - robot ?l - location)
    (connected ?l1 - location ?l2 - location)
    (item_at ?i - item ?l - location)
    (holding ?r - robot ?i - item)
  )
  (:action move
    :parameters (?r - robot ?from - location ?to - location)
    :precondition (and (at ?r ?from) (connected ?from ?to))
    :effect (and (at ?r ?to) (not (at ?r ?from)))
  )
  (:action pick_up
    :parameters (?r - robot ?i - item ?l - location)
    :precondition (and (at ?r ?l) (item_at ?i ?l))
    :effect (and (holding ?r ?i) (not (item_at ?i ?l)))
  )
  (:action drop_off
    :parameters (?r - robot ?i - item ?l - location)
    :precondition (and (at ?r ?l) (holding ?r ?i))
    :effect (and (item_at ?i ?l) (not (holding ?r ?i)))
  )
)