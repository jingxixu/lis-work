# Cook a Meal
Use [pddlstream](https://github.com/caelan/pddlstream) as the planning engine to cook a meal

## Settings
All position are fixed. No motion planning or collision checking. Just planning.
<img width="600" alt="plan map" src="https://github.com/jingxixu/lis-work/blob/master/pddl_examples/cook_meal/images/settings.png">

### Objects
- spoon: scoop salt and oil
- spatula: fry meat and vegetable
- gripper: grasp spoon, spatula, meat, and vegetable
- wok: the place to add salt, oil and fry meat and vegetable

### Rules
- oil and salt can be added to the wok using spoon
- meat and vegetable can be added to the wok using gripper
- oil must be added first, before adding anythong else to the wok
- vegetable can be added and cooked only after meat is added and cooked
- salt must be added last

## Results
<img width="600" alt="results" src="https://github.com/jingxixu/lis-work/blob/master/pddl_examples/cook_meal/images/results.png">
