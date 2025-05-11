# Simulation de Physique Multi-Objets

Cette simulation implémente un moteur de physique 2D avec une détection précise des collisions continues et permet à des objets de rebondir sur divers obstacles selon les lois de la physique. Le projet est construit avec la bibliothèque Raylib pour le rendu graphique.

## Structure du Projet

Le projet est organisé comme suit :

```
bouncing_ball_sim.exe   # Exécutable compilé
nob.c                   # Script de compilation (avec nob.h)
nob.exe                 # Utilitaire de compilation
include/
  common.h              # Définitions des structures et déclarations
lib/                    # Bibliothèques Raylib
  libraylib.a
  libraylibdll.a
  raylib.dll
resources/              # Ressources (sons, etc.)
  bounce.wav
src/
  main.c                # Point d'entrée du programme
  objects.c             # Implémentation des objets et effets
```

## Types d'Objets

### 1. Objets Rebondissants (`BouncingObject`)

Les objets rebondissants sont des cercles qui peuvent se déplacer librement et rebondir sur d'autres objets. Ils sont définis par:

- Position (`Vector2`)
- Vitesse (`Vector2`)
- Rayon (`float`)
- Couleur (`Color`)
- Masse (`float`) - Influence le comportement lors des collisions
- Restitution (`float` entre 0.0 et 1.0) - Facteur de "rebond" (1.0 = rebond parfaitement élastique)

### 2. Objets de Jeu (`GameObject`)

Les objets de jeu sont des formes statiques ou mobiles sur lesquelles les objets rebondissants peuvent rebondir. Ils peuvent avoir différentes formes:

- **Rectangle**: Défini par sa largeur, hauteur, position et couleur
- **Diamant**: Défini par sa diagonale horizontale/verticale, position et couleur

Les objets de jeu peuvent être:
- **Statiques**: Ne se déplacent pas (comme des murs ou des plateformes)
- **Mobiles**: Se déplacent selon une vitesse définie

## Effets de Collision

Un des aspects les plus intéressants de cette simulation est la possibilité d'appliquer différents effets lorsqu'un objet rebondissant entre en collision avec un autre objet. Ces effets peuvent être:

- **Ponctuels**: L'effet est appliqué uniquement au moment précis de la collision
- **Continus**: L'effet est appliqué à chaque frame où les objets sont en contact

### Types d'Effets Disponibles

1. **Changement de Couleur**: Change la couleur de l'objet rebondissant
2. **Boost de Vitesse**: Augmente la vitesse de l'objet rebondissant par un facteur multiplicatif
3. **Ralentissement**: Diminue la vitesse de l'objet rebondissant
4. **Changement de Taille**: Modifie le rayon de l'objet rebondissant
5. **Son**: Joue un effet sonore lors de la collision

## Comment Utiliser le Code

### 1. Création d'Objets Rebondissants

```c
// Créer un objet rebondissant (une balle)
BouncingObject* ball = createBouncingObject(
    (Vector2){SCREEN_WIDTH * 0.3f, SCREEN_HEIGHT * 0.7f}, // Position
    (Vector2){220, -180},                                 // Vitesse
    15,                                                   // Rayon
    RED,                                                  // Couleur
    1.0f,                                                 // Masse
    0.95f                                                 // Restitution (bounciness)
);

// Ajouter l'objet à la liste des objets rebondissants
addBouncingObjectToList(&bouncingObjectList, ball);
```

### 2. Création d'Objets de Jeu

```c
// Créer un rectangle statique
GameObject* rect = createRectangleObject(
    (Vector2){SCREEN_WIDTH * 0.5f, SCREEN_HEIGHT - 50},  // Position
    (Vector2){0, 0},                                    // Vitesse (0 car statique)
    120, 20,                                            // Largeur, hauteur
    SKYBLUE,                                            // Couleur
    true                                                // Statique (true) ou mobile (false)
);

// Créer un diamant mobile
GameObject* diamond = createDiamondObject(
    (Vector2){SCREEN_WIDTH * 0.3f, SCREEN_HEIGHT * 0.3f}, // Position
    (Vector2){40, -20},                                  // Vitesse
    100, 150,                                            // Diagonale horizontale, verticale
    GREEN,                                               // Couleur
    false                                                // Mobile (false)
);

// Ajouter les objets à la liste d'objets de jeu
addObjectToList(&staticObjectList, rect);
addObjectToList(&staticObjectList, diamond);
```

### 3. Création d'Effets de Collision

```c
// Effet de changement de couleur (ponctuel)
CollisionEffect* colorEffect = createColorChangeEffect(BLUE, false);

// Effet de boost de vitesse (continu)
CollisionEffect* speedEffect = createVelocityBoostEffect(1.2f, true);

// Effet de ralentissement (ponctuel)
CollisionEffect* slowEffect = createVelocityDampenEffect(0.8f, false);

// Effet de changement de taille (ponctuel)
CollisionEffect* sizeEffect = createSizeChangeEffect(1.1f, false);

// Effet sonore (ponctuel)
Sound bounceSound = LoadSound("resources/bounce.wav");
CollisionEffect* soundEffect = createSoundPlayEffect(bounceSound, false);
```

### 4. Application d'Effets aux Objets

```c
// Attachement d'un effet à un objet rebondissant
addCollisionEffectsToBouncingObject(ball, colorEffect);

// Attachement d'un effet à un objet de jeu
addCollisionEffectsToGameObject(rect, speedEffect);

// Ou créer directement un objet avec un effet
GameObject* specialRect = createGameObjectWithEffects(
    createRectangleObject((Vector2){400, 300}, (Vector2){0, 0}, 100, 20, RED, true),
    sizeEffect
);
```

### 5. Gestion des Collisions

La gestion des collisions est automatiquement effectuée par la fonction `handleBouncingObjectCollisions()` qui:

1. Détecte les collisions avec précision
2. Calcule le rebond en respectant les lois de la physique
3. Applique les effets de collision appropriés

```c
// Dans la boucle principale de jeu
for (BouncingObject* ball = bouncingObjectList; ball != NULL; ball = ball->next) {
    // Gère les collisions avec tous les objets statiques et mobiles
    handleBouncingObjectCollisions(ball, staticObjectList, dt, 10);
    
    // Applique les collisions avec les bords de l'écran
    applyScreenBoundaryCollisions(ball);
}
```

## Interaction Utilisateur

- **Clic gauche**: Ajoute une nouvelle balle rebondissante avec des propriétés aléatoires à la position du curseur
- **ESC**: Quitte l'application

## Compilation et Exécution

Le projet utilise un système de compilation basé sur [nob.h](https://github.com/tsoding/nobuild) de Tsoding:

```bash
# Compiler nob.c
gcc nob.c -o nob.exe

# Compiler le projet
nob.exe
```

## Détails Techniques Notables

- Détection de collision continue: Calcule le temps exact d'impact pour éviter que les objets ne se traversent même à grande vitesse
- Résolution multi-rebonds: Peut gérer plusieurs rebonds en une seule frame
- Effets de collision modulaires: Système d'effets entièrement extensible

## Comment Étendre le Code

### Ajout d'un Nouveau Type d'Objet

1. Définir la structure de données spécifique à la forme
2. Implémenter les fonctions `render`, `checkCollision`, `update` et `destroy`
3. Créer la fonction de construction (ex: `createNewShapeObject()`)

### Ajout d'un Nouvel Effet de Collision

1. Ajouter un nouveau type dans l'énumération `EffectType`
2. Ajouter une nouvelle structure de paramètres dans l'union `params` de `CollisionEffect`
3. Implémenter la fonction de création (ex: `createNewEffect()`)
4. Mettre à jour la fonction `applyEffects()` pour traiter ce nouvel effet
