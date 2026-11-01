"""Count supporting feet independently of the number of mesh contact points."""
def supporting_feet(model, data):
    feet=set()
    for contact in data.contact:
        pair=[model.geom(contact.geom1).name, model.geom(contact.geom2).name]
        if 'ground' in pair:
            feet.update(n for n in pair if n and n.endswith('_foot_contact'))
    return feet
