---
name: verifiche-facoltative
description: "Feedback 22 nov 2026: una fase di verifica del collaudo che blocca il volo va spiegata subito e, se esiste un ripiego automatico, resa facoltativa invece che obbligatoria"
metadata:
  node_type: memory
  type: feedback
  originSessionId: d1720fc2-19ec-4719-8c66-3bf4a63e3faa
  modified: 2026-11-22T11:58:51.534Z
---

Il 22 nov 2026 l'utente ha chiesto di togliere il giro a piedi (fase 9 del collaudo, allora
obbligatoria prima della missione all'aperto): "a cosa ti servirebbe?". Dopo la spiegazione (soglia
di atterraggio della stima, verifica di FAST-LIO nel campo senza rischiare il drone, registrazione
per tarare FAST-LIO) l'ha voluto di nuovo, ma "opzionale, non obbligatorio". Ora la soglia ha un
ripiego automatico (misura a drone fermo sul decollo), vedi [[missione-aperto-recinto-virtuale]].

**Why:** davanti a una fase obbligatoria di cui non vedeva lo scopo l'utente l'ha tolta; chiarito lo
scopo, l'ha tenuta ma decidendo lui se farla.
**How to apply:** quando propongo una fase o un controllo che impedisce di volare, scrivere subito a
cosa serve; se c'e' un ripiego automatico, renderla facoltativa (in collaudo.py: FACOLTATIVE) e
lasciare all'utente la scelta al campo.
